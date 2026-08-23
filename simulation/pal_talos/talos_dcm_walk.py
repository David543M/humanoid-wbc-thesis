"""
TALOS — marche WBC pilotee par un planificateur DCM (capture point).

Architecture (standard, model-based) :
  PLANIFICATEUR  : pas pre-planifies -> ZMP par appui -> trajectoire DCM
                   (xi) dynamiquement faisable -> reference CoM lissee ;
  WBC-QP         : suit (CoM, orientation, pied de balancement) sous contraintes
                   de contact (le controleur d'equilibre deja valide).

DCM : xi = com + com_dot/omega,  omega=sqrt(g/zc).  Pour un ZMP constant par appui,
xi(t)=zmp+(xi0-zmp)e^{omega t} ; recursion arriere sur les appuis pour xi de debut
de chaque pas ; le CoM suit com_dot=omega(xi-com) (partie STABLE du pendule).

HYBRID RL HOOK (2026-06): set_footstep_offset([dx,dy]) lets an external policy
nudge the NEXT footstep target once per step (DS->SS). This edits ONLY the
planning layer (ZMP plan + DCM recursion). The QP/WBC in control() is untouched.

A placer dans pal_talos/ (a cote de talos_wbc.py).
    python talos_dcm_walk.py            # metriques
    python talos_dcm_walk.py --video    # rend talos_dcm_walk.mp4 (besoin talos_sim.py)
"""
import argparse, numpy as np, mujoco
import talos_wbc as W

T_STEP = 1.3          # duree d'un appui simple (s)
STEP_LEN = 0.10       # avance par pas (m)
N_STEPS = 10
SETTLE = 1.2          # transfert initial du poids (s)
STEP_H = 0.06         # garde au sol du pied (m)
DS_OVL = 0.33         # recouvrement double appui (transfert de poids)
W_SWING = 6e3; KP_SW, KD_SW = 300., 35.
KP_COM_W, KD_COM_W, W_COM_W = 260., 32., 200.
W_POS_W = 8.0
WIDTH_SCALE = 1.0    # multiplie l'ecartement lateral des pas (1.0 = nominal ly)


class DCMWalk(W.WBC):
    def __init__(self, m, d):
        super().__init__(m, d)
        self.left, self.right = self.feet[0], self.feet[1]
        p0 = {fb: d.xpos[fb][:3].copy() for fb in self.feet}
        self.ly = abs(p0[self.left][1])
        self.fz = p0[self.left][2]
        self.zc = float(d.subtree_com[self.base][2])
        self.omega = np.sqrt(9.81 / self.zc)
        # --- plan de pas : suite des appuis (zmp) ---
        self.zmp = []; self.support = []
        rx = p0[self.right][0]; lx = p0[self.left][0]
        side = self.right; ox, oy = rx, -self.ly
        for k in range(N_STEPS):
            self.zmp.append(np.array([ox, oy])); self.support.append(side)
            ox += STEP_LEN
            side = self.left if side == self.right else self.right
            oy = self.ly * WIDTH_SCALE if side == self.left else -self.ly * WIDTH_SCALE
        # --- recursion arriere DCM : xi en debut de chaque appui ---
        E = np.exp(self.omega * T_STEP)
        xi_ini = [None] * N_STEPS
        xi_end = self.zmp[-1].copy()                  # capture finale sur le dernier appui
        for k in range(N_STEPS - 1, -1, -1):
            xi_ini[k] = self.zmp[k] + (xi_end - self.zmp[k]) / E
            xi_end = xi_ini[k]
        self.xi_ini = xi_ini
        # etat de reference CoM (xy), demarre au centre
        self.com_ref_xy = d.subtree_com[self.base][:2].copy()
        self.t = 0.0
        self.swing_from = {self.left: p0[self.left][:3].copy(), self.right: p0[self.right][:3].copy()}
        # --- HYBRID RL HOOK (planning layer only; WBC/QP in control() is untouched) ---
        self.N = N_STEPS
        self._pending_offset = np.zeros(2)     # action for the NEXT footstep, set externally
        self._last_k = -1
        self.Y_MIN = 0.05                      # min |lateral| of a footstep from midline (no foot crossing)
        self.OFF_MAX = np.array([0.06, 0.06])  # bound on |d_foot_x|, |d_foot_y| (m)
        self.nominal_zmp = [z.copy() for z in self.zmp]
        # --- closed-loop DCM/ZMP feedback (model-based, Englsberger capture-point; opt-in) ---
        self.use_dcm_fb = False          # set True to close the loop on measured DCM
        self.k_dcm = 2.5                 # DCM error feedback gain (>0)
        self.fb_clip = 0.05              # clamp on |DCM error| fed back (m), keeps it bounded
        self.use_foot_fb = False         # analytical capture-point foot placement (per-step)
        self.k_foot = 1.0                # capture-point gain (DCM error -> footstep shift)
        self.foot_lat_only = False       # placement capture-point lateral seulement (anti-runaway sagittal)
        self.use_foot_ori = False        # tache d'orientation de pied (semelle a plat)
        self.w_foot_swing = 1500.0; self.w_foot_stance = 0.0   # stance=0 par defaut (flatten le swing seul)
        self.use_cp_swing = False        # continuous: retarget swing foot toward measured capture point
        self.cp_ymax = 0.22              # max |lateral| landing of swing foot (m)
        self.foot_half = np.array([0.09, 0.05])   # support half-size x,y (m) for ZMP clamp
        self.zmp_cmd = None
        self.xi_ref = self.com_ref_xy.copy()
        self.use_hard_contact = True   # no-slip dur sur le pied d'appui (mieux conditionne)
        # --- soft landing (opt-in ; defauts = legacy bit-identique) — mecanismes SEPARES :
        # sl_imp  : impedance d'orientation programmee du pied frais (souple->raide sur T_RAMP),
        #           SANS toucher aux contraintes dures ni a FZ_MIN -> transfert de charge intact
        # sl_ramp : transition de charge (rotation dure differee a mi-rampe + FZ_MIN rampe).
        #           EXPERIMENTAL — desynchronise le transfert vs plan ZMP a switch instantane
        #           (off_lat calibre pour un basculement immediat) : v1 FELL k=7, v2 FELL k=11
        #           (oscillation periode-2 du cycle lateral). Garde pour ablation Ch6.
        self.sl_imp = False
        self.sl_ramp = False
        self.T_RAMP = 0.06             # duree de la rampe / fenetre d'impedance (s)
        self.land_t = {}               # instant du dernier poser, par corps de pied (rempli par la sous-classe)

    # -- external policy sets the offset for the upcoming footstep (applied at next DS->SS) --
    def set_footstep_offset(self, dxy):
        self._pending_offset = np.clip(np.asarray(dxy, float).ravel()[:2],
                                       -self.OFF_MAX, self.OFF_MAX)

    def _recompute_dcm(self):
        """Re-run the existing backward DCM recursion over the (possibly edited) ZMP plan.
        Reuses the same math as __init__ -- no change to the QP/WBC formulation."""
        E = np.exp(self.omega * T_STEP)
        xi_end = self.zmp[-1].copy()
        for j in range(self.N - 1, -1, -1):
            self.xi_ini[j] = self.zmp[j] + (xi_end - self.zmp[j]) / E
            xi_end = self.xi_ini[j]

    def _apply_offset(self, k):
        """At the start of support phase k, commit the pending action to footstep k+1
        (where the current swing foot will land) and refresh the DCM reference."""
        nxt = k + 1
        if nxt < self.N:
            dx, dy = self._pending_offset
            self.zmp[nxt][0] = self.zmp[nxt][0] + dx
            y = self.zmp[nxt][1] + dy
            if self.support[nxt] == self.left:    # left foot in support -> stays on +y side
                y = max(y, self.Y_MIN)
            else:                                  # right foot -> stays on -y side
                y = min(y, -self.Y_MIN)
            self.zmp[nxt][1] = y
            self._recompute_dcm()
        self._pending_offset = np.zeros(2)

    def update(self, dt):
        self.t += dt
        if self.t < SETTLE:                            # transfert initial vers 1er appui
            frac = min(self.t / SETTLE, 1.0)
            tgt = self.zmp[0]
            c0 = self.d.subtree_com[self.base][:2]
            self.com_ref_xy = self.com_ref_xy + (tgt - self.com_ref_xy) * (dt / max(SETTLE - self.t + dt, dt))
            self.k = 0; self.tau = 0.0
            self.phase = "DS"; self.stance = self.support[0]
            self.swing = self.left if self.stance == self.right else self.right
            return
        tg = self.t - SETTLE
        k = int(tg // T_STEP)
        if k >= N_STEPS - 1: k = N_STEPS - 1
        tau = tg - k * T_STEP
        self.k = k; self.tau = tau
        if k != self._last_k:                          # new support phase -> footstep decision
            if self.use_foot_fb:                       # capture-point: shift next foot by DCM error
                _, _, xi_meas = self._measured_dcm()
                delta = self.k_foot * (xi_meas - self.xi_ini[k])
                if self.foot_lat_only: delta[0] = 0.0  # garder l'avance sagittale nominale
                self.set_footstep_offset(delta)        # clamped to OFF_MAX, no foot crossing
            self._apply_offset(k)
            self._last_k = k
        # DCM -> CoM (integration stable)
        xi = self.zmp[k] + (self.xi_ini[k] - self.zmp[k]) * np.exp(self.omega * tau)
        self.xi_ref = xi.copy()
        if self.use_dcm_fb:                            # closed-loop: pull CoM ref to correct measured DCM
            _, _, xi_meas = self._measured_dcm()
            e = np.clip(xi - xi_meas, -self.fb_clip, self.fb_clip)   # bounded DCM error
            xi_cmd = xi + self.k_dcm * e
            self.com_ref_xy = self.com_ref_xy + dt * self.omega * (xi_cmd - self.com_ref_xy)
        else:
            self.com_ref_xy = self.com_ref_xy + dt * self.omega * (xi - self.com_ref_xy)
        self.stance = self.support[k]
        self.swing = self.left if self.stance == self.right else self.right
        # cible pied de balancement = prochain appui (zmp[k+1]) avec cloche en z
        if k + 1 < N_STEPS:
            goal = np.array([self.zmp[k+1][0], self.zmp[k+1][1], self.fz])
        else:
            goal = np.array([self.zmp[k][0], -self.zmp[k][1] if self.stance==self.left else self.ly, self.fz])
        _s_now = min(max(tau / T_STEP, 0), 1)
        if self.use_cp_swing and not (_s_now < DS_OVL or _s_now > 1 - DS_OVL):   # land swing foot under the measured DCM (capture point)
            _, _, xi_meas = self._measured_dcm()
            gy = xi_meas[1]
            if self.swing == self.left:  gy = min(max(gy, self.Y_MIN), self.cp_ymax)
            else:                        gy = max(min(gy, -self.Y_MIN), -self.cp_ymax)
            goal = np.array([goal[0], gy, self.fz])
        s = min(max(tau / T_STEP, 0), 1)
        frm = self.swing_from[self.swing]
        self.swing_pos = (1 - s) * frm + s * goal
        self.swing_pos = self.swing_pos.copy(); self.swing_pos[2] = self.fz + STEP_H * np.sin(np.pi * s)
        # double appui en debut/fin de pas (recouvrement)
        self.phase = "DS" if (s < DS_OVL or s > 1 - DS_OVL) else "SS"
        if s > 1 - DS_OVL:                              # memoriser la pose au moment de poser
            self.swing_from[self.swing] = self.swing_pos.copy()

    def _measured_dcm(self):
        m, d = self.m, self.d
        Jcom = np.zeros((3, m.nv)); mujoco.mj_jacSubtreeCom(m, d, Jcom, self.base)
        com = d.subtree_com[self.base][:2].copy(); com_v = (Jcom @ d.qvel)[:2]
        return com, com_v, com + com_v / self.omega

    def _support_box(self):
        d = self.d
        if self.phase == "DS":
            pl = d.xpos[self.left][:2]; pr = d.xpos[self.right][:2]
            lo = np.minimum(pl, pr) - self.foot_half; hi = np.maximum(pl, pr) + self.foot_half
        else:
            c = d.xpos[self.stance][:2]; lo = c - self.foot_half; hi = c + self.foot_half
        return lo, hi

    def _compute_zmp_cmd(self):
        """Englsberger DCM tracking: p_cmd = p_ref + (1 + k/omega)(xi_meas - xi_ref),
        clamped to the current support polygon (keeps the contact feasible)."""
        _, _, xi_meas = self._measured_dcm()
        p_ref = self.zmp[self.k]
        p = p_ref + (1.0 + self.k_dcm / self.omega) * (xi_meas - self.xi_ref)
        lo, hi = self._support_box()
        self.zmp_cmd = np.clip(p, lo, hi)

    def control(self):
        m, d = self.m, self.d; nv, nu = self.nv, self.nu
        active = list(self.feet) if self.phase == "DS" else [self.stance]
        Jc = self.contact_jac_feet(active); ncp = Jc.shape[0]//3; nf=3*ncp; n=nv+nu+nf
        M=np.zeros((nv,nv)); W.fill_fullM(m,d,M); h=d.qfrc_bias.copy()   # version-agnostic
        Jcom=np.zeros((3,nv)); mujoco.mj_jacSubtreeCom(m,d,Jcom,self.base)
        com=d.subtree_com[self.base]; com_v=Jcom@d.qvel
        ref=np.array([self.com_ref_xy[0], self.com_ref_xy[1], self.zc])
        a_com=KP_COM_W*(ref-com)-KD_COM_W*com_v
        Jori=np.zeros((3,nv)); tmp=np.zeros((3,nv)); mujoco.mj_jacBody(m,d,tmp,Jori,self.base)
        q=d.qpos[3:7]; err=np.zeros(3); neg=np.zeros(4); res=np.zeros(4)
        mujoco.mju_negQuat(neg,q); mujoco.mju_mulQuat(res,np.array([1.,0,0,0]),neg); mujoco.mju_quat2Vel(err,res,1.)
        a_ori=W.KP_ORI*err-W.KD_ORI*(Jori@d.qvel)
        Jpos=np.zeros((nu,nv))
        for kk,dof in enumerate(self.act_dofs): Jpos[kk,dof]=1.
        qcur=np.array([d.qpos[a] for a in self.act_qadr]); vcur=np.array([d.qvel[a] for a in self.act_dofs])
        a_pos=W.KP_POS*(self.home-qcur)-W.KD_POS*vcur
        G=np.zeros((n,n)); a=np.zeros(n)
        def add(J,rhs,w): G[:nv,:nv]+=2*w*(J.T@J); a[:nv]+=2*w*(J.T@rhs)
        add(Jcom,a_com,W_COM_W); add(Jori,a_ori,W.W_ORI); add(Jpos,a_pos,W_POS_W)
        if not self.use_hard_contact:
            add(Jc,-W.BAUMGARTE*(Jc@d.qvel),W.W_CONTACT)
        if self.phase=="SS":
            fb=self.swing; p=d.xpos[fb]; Jsw=np.zeros((3,nv)); mujoco.mj_jac(m,d,Jsw,None,p,fb)
            a_sw=KP_SW*(self.swing_pos-p)-KD_SW*(Jsw@d.qvel); add(Jsw,a_sw,W_SWING)
        # --- soft landing : facteur de rampe post-poser par pied (1.0 = contact etabli / legacy) ---
        ramp={fb:1.0 for fb in active}
        if self.sl_imp or self.sl_ramp:
            for fb in active:
                ramp[fb]=float(np.clip((self.t-self.land_t.get(fb,-1e9))/max(self.T_RAMP,1e-6),0.0,1.0))
        if self.use_foot_ori:                          # garder les semelles a plat
            if self.w_foot_stance>0:
                for fb in active:                      # pied(s) en contact -> raidir (anti-roulis)
                    if self.sl_imp and ramp[fb]<1.0:   # impedance programmee : souple a l'impact, sur-amortie
                        Jr,a_o=self.foot_ori_task(fb, W.KP_FOOT*(0.5+0.5*ramp[fb]),
                                                  W.KD_FOOT*(1.3-0.3*ramp[fb]))
                        add(Jr,a_o,max(self.w_foot_stance,300.0))
                    else:
                        Jr,a_o=self.foot_ori_task(fb); add(Jr,a_o,self.w_foot_stance)
            if self.phase=="SS":                       # pied de balancement -> le poser a plat
                Jr,a_o=self.foot_ori_task(self.swing); add(Jr,a_o,self.w_foot_swing)
        G[:nv,:nv]+=2*W.EPS_QDD*np.eye(nv); G[nv:nv+nu,nv:nv+nu]+=2*W.EPS_TAU*np.eye(nu); G[nv+nu:,nv+nu:]+=2*W.EPS_F*np.eye(nf)
        Aeq=np.zeros((nv,n)); Aeq[:,:nv]=M; Aeq[:,nv:nv+nu]=-self.S; Aeq[:,nv+nu:]=-Jc.T; beq=-h
        if self.use_hard_contact:                      # no-slip dur sur le(s) pied(s) en contact (active seulement)
            cr,cb=[],[]
            for fb in active:
                Jp6=np.zeros((3,nv)); Jr6=np.zeros((3,nv)); mujoco.mj_jac(m,d,Jp6,Jr6,d.xpos[fb],fb)
                # sl_ramp seulement : rotation en cout mou pendant la 1ere moitie de la rampe
                # (sl_imp seul NE touche PAS aux contraintes dures -> autorite ZMP immediate)
                Js=(Jp6,Jr6) if (not self.sl_ramp or ramp[fb]>=0.5) else (Jp6,)
                for J in Js:
                    row=np.zeros((3,n)); row[:,:nv]=J; cr.append(row); cb.append(-W.BAUMGARTE*(J@d.qvel))
                if self.sl_ramp and ramp[fb]<0.5 and not (self.use_foot_ori and self.w_foot_stance>0):
                    Jr_,a_=self.foot_ori_task(fb, W.KP_FOOT*(0.5+0.5*ramp[fb]),
                                              W.KD_FOOT*(1.3-0.3*ramp[fb]))
                    add(Jr_,a_,300.0)
            Aeq=np.vstack([Aeq]+cr); beq=np.concatenate([beq]+cb)
        rows,lb=[],[]
        ci=0
        for fb in active:
            fzmin=W.FZ_MIN*(ramp[fb] if self.sl_ramp else 1.0)   # rampe de charge (sl_ramp seulement)
            for _cl in self.corners_local[fb]:
                b=nv+nu+3*ci
                v=np.zeros(n); v[b+2]=1.; rows.append(v); lb.append(fzmin)
                for sg in(-1.,1.):
                    v=np.zeros(n); v[b+2]=W.MU; v[b+0]=sg; rows.append(v); lb.append(0.)
                    v=np.zeros(n); v[b+2]=W.MU; v[b+1]=sg; rows.append(v); lb.append(0.)
                ci+=1
        for j in range(nu):
            v=np.zeros(n); v[nv+j]=1.; rows.append(v); lb.append(self.tau_min[j])
            v=np.zeros(n); v[nv+j]=-1.; rows.append(v); lb.append(-self.tau_max[j])
        try:
            x=W.solve_qp(G,a,Aeq,beq,np.vstack(rows),np.array(lb)); tau=x[nv:nv+nu]
        except Exception:
            tau=np.array([h[dof] for dof in self.act_dofs])+W.KP_POS*(self.home-qcur)-W.KD_POS*vcur
            self._qp_fail=getattr(self,"_qp_fail",0)+1
        d.ctrl[:]=np.clip(tau,self.tau_min,self.tau_max)

    def contact_jac_feet(self, feet):
        m,d=self.m,self.d; rows=[]
        for fb in feet:
            R=d.xmat[fb].reshape(3,3); p=d.xpos[fb]
            for cl in self.corners_local[fb]:
                Jp=np.zeros((3,m.nv)); mujoco.mj_jac(m,d,Jp,None,p+R@cl,fb); rows.append(Jp)
        return np.vstack(rows)


def init_limit_cycle(c, d):
    """Lance l'etat sur le cycle limite lateral periodique :
    xi_s = ly*tanh(omega*T/2) ; vitesse laterale d'entree = omega*(xi_s,0 - c_y)."""
    global SETTLE
    xi_s = c.ly * np.tanh(c.omega * T_STEP / 2.0)
    sign0 = np.sign(c.zmp[0][1])                  # cote du 1er appui (droite = -1)
    cy = float(d.subtree_com[c.base][1])
    d.qvel[1] = c.omega * (sign0 * xi_s - cy)     # impose le DCM lateral d'entree
    c.com_ref_xy[1] = cy
    mujoco.mj_forward(c.m, d)
    SETTLE = 0.30                                  # transfert court : deja sur le cycle
    print("[limit-cycle] xi_s=%.4f m  vit.laterale entree=%.3f m/s" % (xi_s, d.qvel[1]))


def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--video",action="store_true"); ap.add_argument("--dur",type=float,default=9.0)
    ap.add_argument("--feedback",action="store_true",help="enable closed-loop DCM/ZMP feedback")
    ap.add_argument("--kdcm",type=float,default=2.5)
    ap.add_argument("--footfb",action="store_true",help="analytical capture-point foot placement")
    ap.add_argument("--kfoot",type=float,default=1.0)
    ap.add_argument("--cpswing",action="store_true",help="continuous capture-point swing retargeting")
    ap.add_argument("--tstep",type=float,default=None); ap.add_argument("--steplen",type=float,default=None)
    ap.add_argument("--dsovl",type=float,default=None)
    ap.add_argument("--walk-cl",dest="walk_cl",action="store_true",help="preset: best closed-loop walk (fast steps + capture-point + DCM feedback)")
    ap.add_argument("--offmax",type=float,default=0.10,help="footstep offset bound (m) for capture-point")
    ap.add_argument("--limit-cycle",dest="limit_cycle",action="store_true",help="initialise l'etat sur le cycle limite lateral")
    ap.add_argument("--footori",dest="footori",action="store_true",help="tache d'orientation de pied (semelle a plat)")
    a=ap.parse_args()
    global T_STEP, STEP_LEN, DS_OVL
    if a.walk_cl:
        a.feedback=a.footfb=a.cpswing=True; a.kdcm=1.0; a.kfoot=1.0; a.offmax=0.20
        if a.tstep is None: a.tstep=0.5
        if a.steplen is None: a.steplen=0.05
        if a.dsovl is None: a.dsovl=0.45
    if a.tstep   is not None: T_STEP=a.tstep
    if a.steplen is not None: STEP_LEN=a.steplen
    if a.dsovl   is not None: DS_OVL=a.dsovl
    if W.quadprog is None: raise SystemExit("pip install quadprog")
    m=mujoco.MjModel.from_xml_path(W.MODEL); d=mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m,d,0); mujoco.mj_forward(m,d)
    c=DCMWalk(m,d); c.use_dcm_fb=a.feedback; c.k_dcm=a.kdcm
    c.use_foot_fb=a.footfb; c.k_foot=a.kfoot
    if a.footfb: c.OFF_MAX=np.array([a.offmax, a.offmax])   # capture-point needs more lateral authority
    c.use_cp_swing=a.cpswing
    c.use_foot_ori=getattr(a,'footori',False)
    if getattr(a,"limit_cycle",False): init_limit_cycle(c, d)
    dt=m.opt.timestep; render=None
    if a.video:
        import talos_sim; render=talos_sim.render
    n=int(a.dur/dt); fe=max(1,int(1/(30*dt))); frames=[]; last=-1
    for i in range(n):
        c.update(dt); c.control(); mujoco.mj_step(m,d)
        if c.t>SETTLE and getattr(c,'k',0)!=last:
            last=c.k
        if a.video and i%fe==0: frames.append(render(m,d))
    print("end z=%.3f x=%.3f reached_step=%d %s | QP fails=%d"%(
        d.qpos[2],d.qpos[0],getattr(c,'k',0),"UPRIGHT" if d.qpos[2]>0.8 else "FELL",getattr(c,'_qp_fail',0)))
    if a.video:
        import imageio.v2 as imageio; imageio.mimsave("talos_dcm_walk.mp4",frames,fps=30); print("[ok] talos_dcm_walk.mp4")


if __name__=="__main__": main()

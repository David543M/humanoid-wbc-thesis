"""
setup_github_repo.py
====================
Creates the GitHub repository 'humanoid-wbc-thesis' under github.com/David543M
and pushes the current IRP folder content to it.

Usage (from the IRP folder):
    python setup_github_repo.py

Requirements:
    pip install requests
    git must be installed and in PATH
"""

import os
import sys
import subprocess
import requests

# ── Configuration ────────────────────────────────────────────────────────────
# Set your token via environment variable: set GITHUB_TOKEN=ghp_...
# Never hardcode tokens in versioned files.
GITHUB_TOKEN    = os.environ.get("GITHUB_TOKEN", "")
GITHUB_USERNAME = "David543M"
REPO_NAME       = "humanoid-wbc-thesis"
REPO_DESC       = (
    "Master's Thesis — Development and Simulation of a "
    "Whole-Body Control Framework for a Humanoid Robot"
)
REPO_PRIVATE    = False   # Set to True if you want a private repo
# ─────────────────────────────────────────────────────────────────────────────

HEADERS = {
    "Authorization": f"token {GITHUB_TOKEN}",
    "Accept": "application/vnd.github.v3+json",
}

def run(cmd, cwd=None, check=True):
    """Run a shell command, print it, and return CompletedProcess."""
    print(f"  $ {cmd}")
    result = subprocess.run(cmd, shell=True, cwd=cwd, capture_output=True, text=True)
    if result.stdout.strip():
        print(f"    {result.stdout.strip()}")
    if result.stderr.strip():
        print(f"    [stderr] {result.stderr.strip()}")
    if check and result.returncode != 0:
        print(f"\n❌ Command failed (exit {result.returncode}). Aborting.")
        sys.exit(1)
    return result

def create_repo():
    """Create the GitHub repository via the API."""
    print(f"\n📡 Creating repository '{REPO_NAME}' on GitHub...")
    url = "https://api.github.com/user/repos"
    payload = {
        "name": REPO_NAME,
        "description": REPO_DESC,
        "private": REPO_PRIVATE,
        "auto_init": False,
    }
    resp = requests.post(url, json=payload, headers=HEADERS, timeout=30)

    if resp.status_code == 201:
        data = resp.json()
        print(f"✅ Repository created: {data['html_url']}")
        return data["clone_url"], data["html_url"]
    elif resp.status_code == 422:
        # Repo already exists
        print("⚠️  Repository already exists on GitHub — continuing with push.")
        clone_url = f"https://github.com/{GITHUB_USERNAME}/{REPO_NAME}.git"
        html_url  = f"https://github.com/{GITHUB_USERNAME}/{REPO_NAME}"
        return clone_url, html_url
    else:
        print(f"❌ GitHub API error {resp.status_code}: {resp.text}")
        sys.exit(1)

def set_repo_topics():
    """Add relevant topics/tags to the repo."""
    print("\n🏷️  Setting repository topics...")
    url = f"https://api.github.com/repos/{GITHUB_USERNAME}/{REPO_NAME}/topics"
    payload = {
        "names": [
            "humanoid-robot", "whole-body-control", "robotics",
            "mujoco", "pinocchio", "quadratic-programming",
            "locomotion", "loco-manipulation", "masters-thesis", "simulation"
        ]
    }
    headers = {**HEADERS, "Accept": "application/vnd.github.mercy-preview+json"}
    resp = requests.put(url, json=payload, headers=headers, timeout=30)
    if resp.status_code == 200:
        print("✅ Topics set.")
    else:
        print(f"⚠️  Could not set topics ({resp.status_code}) — continuing.")

def init_and_push(clone_url, irp_dir):
    """Initialise git in the IRP folder and push to GitHub."""
    git_dir = os.path.join(irp_dir, ".git")

    if os.path.isdir(git_dir):
        print("\n📂 Git already initialised in this folder.")
    else:
        print("\n📂 Initialising git repository...")
        run("git init", cwd=irp_dir)
        run("git checkout -b main", cwd=irp_dir)

    # Configure remote
    result = run("git remote get-url origin", cwd=irp_dir, check=False)
    if result.returncode == 0:
        print("⚠️  Remote 'origin' already set — updating URL.")
        run(f"git remote set-url origin {clone_url}", cwd=irp_dir)
    else:
        run(f"git remote add origin {clone_url}", cwd=irp_dir)

    # Configure credentials in the remote URL
    auth_url = clone_url.replace(
        "https://", f"https://{GITHUB_USERNAME}:{GITHUB_TOKEN}@"
    )
    run(f"git remote set-url origin {auth_url}", cwd=irp_dir)

    # Create .gitignore if missing
    gitignore_path = os.path.join(irp_dir, ".gitignore")
    if not os.path.exists(gitignore_path):
        print("\n📝 Creating .gitignore...")
        with open(gitignore_path, "w") as f:
            f.write(GITIGNORE_CONTENT)

    # Stage, commit, push
    print("\n📤 Staging all files...")
    run("git add -A", cwd=irp_dir)

    result = run('git diff --cached --quiet', cwd=irp_dir, check=False)
    if result.returncode == 0:
        print("ℹ️  Nothing new to commit.")
    else:
        run('git commit -m "Initial commit — thesis workspace structure and master memory"', cwd=irp_dir)

    print("\n🚀 Pushing to GitHub...")
    run("git push -u origin main", cwd=irp_dir)


GITIGNORE_CONTENT = """\
# Python
__pycache__/
*.py[cod]
*.egg-info/
.venv/
env/
dist/
build/

# Jupyter
.ipynb_checkpoints/

# Simulation artefacts
*.bag
*.log
*.csv.bak

# OS
.DS_Store
Thumbs.db
desktop.ini

# Large binary files (add manually if needed)
*.pkl
*.h5
*.hdf5
*.pt
*.pth

# Secrets (never commit tokens)
*.env
.env*
secrets.py
"""


def main():
    # Locate the IRP folder (same directory as this script)
    irp_dir = os.path.dirname(os.path.abspath(__file__))
    print(f"📁 Working directory: {irp_dir}")

    # Step 1 — Create repo
    clone_url, html_url = create_repo()

    # Step 2 — Add topics
    set_repo_topics()

    # Step 3 — Init git and push
    init_and_push(clone_url, irp_dir)

    print(f"\n🎉 Done! Your thesis repo is live at:\n   {html_url}\n")


if __name__ == "__main__":
    main()

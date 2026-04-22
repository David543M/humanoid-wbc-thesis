# ============================================================
# cleanup_github.ps1
# Remove AI/tooling files already committed to GitHub.
# Apply updated .gitignore. Push clean state.
#
# PREREQUISITE: Close VS Code, GitHub Desktop, any git client
#               before running this script.
#
# USAGE: In PowerShell, from C:\Users\david\IRP:
#   .\cleanup_github.ps1
# ============================================================

Set-Location $PSScriptRoot

Write-Host ""
Write-Host "=== Step 1: Git status ===" -ForegroundColor Cyan
git status --short
if ($LASTEXITCODE -ne 0) {
    Write-Host "Git error -- make sure no git client is open." -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "=== Step 2: Untrack AI/tooling folders (files kept locally) ===" -ForegroundColor Cyan

$foldersToUntrack = @(
    "01_prompts",
    "02_raw_research",
    "03_processed",
    "04_council_analysis",
    "05_targeted_research",
    "13_logbook"
)

foreach ($folder in $foldersToUntrack) {
    if (Test-Path $folder) {
        Write-Host "  Untracking: $folder/" -ForegroundColor Yellow
        git rm -r --cached "$folder/"
    }
}

$filesToUntrack = @(
    "master_memory.md",
    "setup_github_repo.py",
    "07_review/adversarial_review.md"
)

foreach ($file in $filesToUntrack) {
    if (Test-Path $file) {
        Write-Host "  Untracking: $file" -ForegroundColor Yellow
        git rm --cached "$file"
    }
}

Write-Host ""
Write-Host "=== Step 3: Stage updated files ===" -ForegroundColor Cyan
git add .gitignore
git add cleanup_github.ps1

Write-Host ""
Write-Host "=== Step 4: Commit ===" -ForegroundColor Cyan
git commit -m "[fix] Enforce GitHub scope: untrack AI tooling folders; update .gitignore and master_memory s20"

if ($LASTEXITCODE -ne 0) {
    Write-Host "Commit failed -- nothing to commit or error." -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "=== Step 5: Push to GitHub ===" -ForegroundColor Cyan
git push origin main

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "OK -- Cleanup done. GitHub no longer contains AI tooling files." -ForegroundColor Green
    Write-Host "     Local files are untouched -- only removed from git tracking." -ForegroundColor Green
    Write-Host ""
} else {
    Write-Host ""
    Write-Host "FAILED -- Push error. Check your GitHub access (PAT token)." -ForegroundColor Red
}

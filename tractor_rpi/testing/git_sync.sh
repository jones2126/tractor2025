#!/bin/bash
# git_sync.sh — Pull latest code from GitHub on the tractor RPi
# Run from anywhere: bash ~/tractor2025/tractor_rpi/testing/git_sync.sh
# Or make executable: chmod +x git_sync.sh then ./git_sync.sh

REPO_DIR="$HOME/tractor2025"

echo "=== Tractor RPi Git Sync ==="
echo "Repo: $REPO_DIR"
echo ""

# Confirm repo exists
if [ ! -d "$REPO_DIR/.git" ]; then
    echo "ERROR: $REPO_DIR is not a git repository."
    exit 1
fi

cd "$REPO_DIR"

# Show current branch and status before pulling
echo "Branch: $(git branch --show-current)"
echo ""

# Check for uncommitted local changes
if ! git diff --quiet || ! git diff --cached --quiet; then
    echo "WARNING: You have uncommitted local changes:"
    git status --short
    echo ""
    read -p "Continue with pull anyway? (y/n): " confirm
    if [ "$confirm" != "y" ]; then
        echo "Aborted. Commit or stash your changes first."
        exit 1
    fi
    echo ""
fi

# Pull from GitHub
echo "Pulling from GitHub..."
git pull

echo ""
echo "=== Sync complete ==="
echo "Current status:"
git status --short

#!/bin/bash

# Define color variables
RED='\033[0;31m'
GREEN='\033[0;32m'  # Green color
BLUE='\033[0;34m'  # Blue color
NC='\033[0m' # No Color (reset to default)

# Ensure the script works from any location within the repo by using the top-level directory
# Move to the repository's root
repo_root=$(git rev-parse --show-toplevel 2>/dev/null)

echo -e "${BLUE}Setting up git aliases...${NC}"

# Check if this is inside a git repository
if [ $? -ne 0 ]; then
    echo "Error: This is not a Git repository (no .git directory found)."
    exit 1
fi

# Path to the .git/config file and .gitconfig file
git_config_file="$repo_root/.git/config"
gitconfig_file="$repo_root/.gitconfig" 

# Check if the .gitconfig file exists in the specified location
if [ ! -f "$gitconfig_file" ]; then
    echo "Error: .gitconfig file not found at $gitconfig_file."
    exit 1
fi

# Check if the .git/config file exists
if [ ! -f "$git_config_file" ]; then
    echo "Error: .git/config file not found."
    exit 1
fi

# Check if the [alias] section is present or any aliases exist already
if grep -q '^\[alias\]' "$git_config_file"; then
    echo ".git/config already contains the [alias] section. Skipping append."
    echo -e "${RED}The \`git subup\` command might not work...${NC}"
else
    echo "Appending .gitconfig to .git/config..."
    cat "$gitconfig_file" >> "$git_config_file"
    echo -e "${GREEN}Successfully appended .gitconfig to .git/config.${NC}"
fi
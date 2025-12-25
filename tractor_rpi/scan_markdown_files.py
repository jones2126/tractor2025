#!/usr/bin/env python3
"""
scan_markdown_files.py
Scans local git repository for .md files and compares them with tracked GitHub URLs.
Helps identify missing documentation in tracking list.
"""

import os
import subprocess
from pathlib import Path
from typing import Set, Dict, List
import argparse
import csv

# Your tracked GitHub URLs - Project Documentation Only
TRACKED_URLS = [
    # Original tracked URLs
    "https://github.com/jones2126/tractor2025/blob/main/README.md",
    "https://github.com/jones2126/tractor2025/blob/main/BridgevilleRTKBase/README.md",
    "https://github.com/jones2126/tractor2025/blob/main/BridgevilleRTKBase/shared_README.md",
    "https://github.com/jones2126/tractor2025/blob/main/tractor_rpi/README.md",
    "https://github.com/jones2126/tractor2025/blob/main/tractor_rpi/f9p/readme.md",
    "https://github.com/jones2126/tractor2025/blob/main/tractor_rpi/light-tower/LED_STATUS_TOWER_README.md",
    "https://github.com/jones2126/tractor2025/tree/main/tractor_rpi/oak-camera/readme.md",
    "https://github.com/jones2126/tractor2025/blob/2f10b0e6e60cc389181fc46f4a9c154a07645f0b/tractor_rpi/teensy_serial_bridge_documentation.md",
    "https://github.com/jones2126/tractor2025/blob/2f10b0e6e60cc389181fc46f4a9c154a07645f0b/tractor_teensy/tractor_teensy_main_documentation.md",
    
    # Newly added project documentation
    "https://github.com/jones2126/tractor2025/blob/main/BridgevilleRTKBase/esp32/production/esp32-renogy-csv-logger/readme.md",
    "https://github.com/jones2126/tractor2025/blob/main/BridgevilleRTKBase/raspberry-pi/archive/f9p-setup/ZED-F9P_Firmware_Update.md",
    "https://github.com/jones2126/tractor2025/blob/main/BridgevilleRTKBase/shared/docs/README.md",
    "https://github.com/jones2126/tractor2025/blob/main/BridgevilleRTKBase/shared/docs/geodetic_to_ecef_and_back.md",
    "https://github.com/jones2126/tractor2025/blob/main/tractor_rpi/light-tower/LED_QUICK_REFERENCE.md",
    "https://github.com/jones2126/tractor2025/blob/main/tractor_rpi/oak-camera/requirements_doc.md",
    "https://github.com/jones2126/tractor2025/blob/main/tractor_rpi/testing/oak_camera_test/README.md",
    "https://github.com/jones2126/tractor2025/blob/main/tractor_teensy/MESSAGE_FORMAT_REFERENCE.md"
]

def get_repo_root() -> Path:
    """Find the git repository root directory."""
    try:
        result = subprocess.run(
            ['git', 'rev-parse', '--show-toplevel'],
            capture_output=True,
            text=True,
            check=True
        )
        return Path(result.stdout.strip())
    except subprocess.CalledProcessError:
        raise RuntimeError("Not in a git repository or git not installed")

def find_markdown_files(repo_root: Path) -> Set[Path]:
    """Recursively find all .md files in the repository."""
    markdown_files = set()
    
    # Directories to ignore
    ignore_dirs = {'.git', 'node_modules', 'venv', '__pycache__', '.venv', 'build'}
    
    for root, dirs, files in os.walk(repo_root):
        # Remove ignored directories from search
        dirs[:] = [d for d in dirs if d not in ignore_dirs]
        
        # Skip any path that contains .pio
        if '.pio' in root:
            continue
        
        for file in files:
            if file.endswith('.md'):
                full_path = Path(root) / file
                relative_path = full_path.relative_to(repo_root)
                markdown_files.add(relative_path)
    
    return markdown_files

def extract_path_from_url(url: str) -> str:
    """Extract the file path from a GitHub URL."""
    # Handle blob URLs (most common)
    if '/blob/' in url:
        # Split on /blob/ and take everything after the branch name
        parts = url.split('/blob/')
        if len(parts) == 2:
            # Remove the branch/commit hash (first path component after blob)
            path_parts = parts[1].split('/', 1)
            if len(path_parts) == 2:
                return path_parts[1]
    
    # Handle tree URLs (directories, but might have .md)
    if '/tree/' in url:
        parts = url.split('/tree/')
        if len(parts) == 2:
            path_parts = parts[1].split('/', 1)
            if len(path_parts) == 2:
                return path_parts[1]
    
    return None

def normalize_path(path: str) -> str:
    """Normalize path for comparison (handle case-insensitive filesystems)."""
    return str(Path(path)).lower()

def compare_files(local_files: Set[Path], tracked_urls: List[str]) -> Dict:
    """Compare local markdown files with tracked URLs."""
    
    # Extract paths from URLs
    tracked_paths = set()
    for url in tracked_urls:
        path = extract_path_from_url(url)
        if path:
            tracked_paths.add(normalize_path(path))
    
    # Normalize local paths
    local_normalized = {normalize_path(str(p)) for p in local_files}
    
    # Find differences
    missing_from_tracking = local_normalized - tracked_paths
    tracked_but_not_local = tracked_paths - local_normalized
    
    # Convert back to original paths for reporting
    missing_files = {p for p in local_files if normalize_path(str(p)) in missing_from_tracking}
    
    return {
        'total_local': len(local_files),
        'total_tracked': len(tracked_paths),
        'missing_from_tracking': missing_files,
        'tracked_but_not_local': tracked_but_not_local,
        'all_match': len(missing_from_tracking) == 0 and len(tracked_but_not_local) == 0
    }

def generate_github_url(file_path: Path, branch: str = 'main') -> str:
    """Generate a GitHub URL for a given file path."""
    repo_base = "https://github.com/jones2126/tractor2025/blob"
    return f"{repo_base}/{branch}/{file_path}"

def print_report(comparison: Dict, repo_root: Path):
    """Print a formatted report of the comparison."""
    print("\n" + "="*80)
    print("MARKDOWN FILE SCAN REPORT")
    print("="*80)
    print(f"\nRepository: {repo_root}")
    print(f"Total local .md files found: {comparison['total_local']}")
    print(f"Total tracked URLs: {comparison['total_tracked']}")
    
    if comparison['all_match']:
        print("\n✓ All local markdown files are tracked!")
    else:
        if comparison['missing_from_tracking']:
            print("\n" + "-"*80)
            print("⚠ MISSING FROM TRACKING LIST:")
            print("-"*80)
            for file_path in sorted(comparison['missing_from_tracking']):
                print(f"\n  Local file: {file_path}")
                print(f"  GitHub URL: {generate_github_url(file_path)}")
        
        if comparison['tracked_but_not_local']:
            print("\n" + "-"*80)
            print("⚠ TRACKED BUT NOT FOUND LOCALLY:")
            print("-"*80)
            for path in sorted(comparison['tracked_but_not_local']):
                print(f"  {path}")
                print("  (May have been moved, renamed, or is on a different branch)")
    
    print("\n" + "="*80)

def save_cross_reference(local_files: Set[Path], tracked_urls: List[str], 
                         repo_root: Path, output_file: Path):
    """Create a cross-reference CSV between URLs and local file paths."""
    
    # Build mapping of normalized paths to actual local paths
    local_path_map = {normalize_path(str(p)): p for p in local_files}
    
    # Extract paths from URLs
    url_to_path = {}
    for url in tracked_urls:
        extracted_path = extract_path_from_url(url)
        if extracted_path:
            url_to_path[url] = extracted_path
    
    # Create cross-reference data
    cross_ref_data = []
    
    for url in sorted(tracked_urls):
        path_from_url = url_to_path.get(url, "")
        normalized = normalize_path(path_from_url) if path_from_url else ""
        local_path = local_path_map.get(normalized, None)
        
        if local_path:
            full_local_path = repo_root / local_path
            exists = "✓" if full_local_path.exists() else "✗"
        else:
            full_local_path = "NOT FOUND"
            exists = "✗"
        
        cross_ref_data.append({
            'Status': exists,
            'GitHub URL': url,
            'File Path (from URL)': path_from_url,
            'Local File Path': str(full_local_path),
            'Exists': 'Yes' if exists == "✓" else 'No'
        })
    
    # Write to CSV
    with open(output_file, 'w', newline='') as f:
        if cross_ref_data:
            fieldnames = ['Status', 'GitHub URL', 'File Path (from URL)', 
                         'Local File Path', 'Exists']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(cross_ref_data)
    
    print(f"\n✓ Cross-reference saved to: {output_file}")
    
    # Print summary
    found = sum(1 for item in cross_ref_data if item['Exists'] == 'Yes')
    total = len(cross_ref_data)
    print(f"   Found: {found}/{total} tracked files")

def save_urls_to_file(comparison: Dict, output_file: Path):
    """Save missing URLs to a file for easy addition to tracking list."""
    if not comparison['missing_from_tracking']:
        print(f"\nNo missing files to save.")
        return
    
    with open(output_file, 'w') as f:
        f.write("# Missing markdown files found in repository\n")
        f.write("# Add these URLs to your tracking list\n\n")
        for file_path in sorted(comparison['missing_from_tracking']):
            url = generate_github_url(file_path)
            f.write(f"{url}\n")
    
    print(f"\n✓ Missing URLs saved to: {output_file}")

def main():
    parser = argparse.ArgumentParser(
        description="Scan repository for .md files and compare with tracked URLs"
    )
    parser.add_argument(
        '--repo-path',
        type=Path,
        help='Path to repository (default: current directory)'
    )
    parser.add_argument(
        '--save-missing',
        type=Path,
        default='missing_markdown_urls.txt',
        help='Output file for missing URLs (default: missing_markdown_urls.txt)'
    )
    parser.add_argument(
        '--cross-reference',
        type=Path,
        default='markdown_cross_reference.csv',
        help='Output CSV file for URL/path cross-reference (default: markdown_cross_reference.csv)'
    )
    parser.add_argument(
        '--list-all',
        action='store_true',
        help='List all markdown files found'
    )
    
    args = parser.parse_args()
    
    try:
        # Determine repository root
        if args.repo_path:
            repo_root = args.repo_path.resolve()
            os.chdir(repo_root)
        else:
            repo_root = get_repo_root()
        
        print(f"Scanning repository: {repo_root}")
        print(f"Ignoring all paths containing '.pio'")
        
        # Find all markdown files
        local_files = find_markdown_files(repo_root)
        
        if args.list_all:
            print("\nAll markdown files found (excluding .pio directories):")
            for f in sorted(local_files):
                print(f"  {f}")
        
        # Compare with tracked URLs
        comparison = compare_files(local_files, TRACKED_URLS)
        
        # Print report
        print_report(comparison, repo_root)
        
        # Save cross-reference
        cross_ref_path = repo_root / args.cross_reference
        save_cross_reference(local_files, TRACKED_URLS, repo_root, cross_ref_path)
        
        # Save missing URLs to file if any found
        if comparison['missing_from_tracking']:
            output_path = repo_root / args.save_missing
            save_urls_to_file(comparison, output_path)
        
    except Exception as e:
        print(f"Error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())

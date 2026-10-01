# Repository Data-Retention Policy

Status: **Approved 2026-10-01**

## Purpose

Keep the code, documentation, reviewed site knowledge, and runnable mission
packages needed to reproduce tractor work in GitHub. Keep high-volume raw run
captures local by default.

## Store in GitHub

- Production and test source code.
- Reusable collection, analysis, validation, and build scripts under
  `field_testing/tools/`.
- Project documentation and dated engineering notes under `obsidian_vault/`,
  except machine-specific Obsidian state and regenerable indexes.
- Confirmed analysis under `field_testing/sites/<site>/analysis/`, including
  the report, lightweight plots, machine-readable results, source hashes, and
  enough provenance to reproduce the result.
- Canonical and revisioned site boundary, obstacle, and inventory data under
  `field_testing/sites/<site>/site_inventory/`.
- Pure-pursuit mission commands, launchers, validators, audit reports, and
  operator instructions under `field_testing/sites/<site>/mission_plans/`.
- Small configuration backups, manifests, and checksums needed to explain or
  reproduce a confirmed field result.

`REVIEW_ONLY` means that an artifact is not authorized for field operation. It
does not, by itself, mean the artifact should remain outside Git. Review-only
artifacts that are inputs to later work or are needed for traceability should
be committed with their status clearly labeled.

## Keep local by default

- Raw field and pursuit telemetry, bulk CSV/JSON/NDJSON captures, videos, and
  other high-volume run data under `field_testing/sites/<site>/runs/`.
- Temporary exports, scratch calculations, caches, dependency directories,
  compiled firmware, and other reproducible build output.
- Secrets, credentials, access tokens, and machine-specific configuration.

When a raw run supports a confirmed conclusion, commit a compact evidence
package under the site's `analysis/` directory. That package should identify
the local source filenames, record SHA-256 hashes, describe the analysis tool
and command, and contain the reviewed result. Raw data may be deliberately
promoted to Git when it is small, essential, and explicitly reviewed.

## End-of-session workflow

1. List every file that is local but neither tracked nor ignored:

   ```powershell
   git ls-files --others --exclude-standard | Sort-Object
   ```

2. Commit confirmed code, tools, documentation, analysis, site inventory, and
   mission packages.
3. Confirm raw run data has a local backup and, when relevant, a checksum and
   provenance record in Git.
4. Check the staged file list and diff before committing.
5. Push, fetch, and confirm the branch is zero commits ahead and behind its
   GitHub branch.

## `.gitignore` rules

The repository uses the following narrow rules for local field data:

```gitignore
# Raw field-run captures are local by default. Existing tracked files remain
# tracked; deliberately promote a reviewed exception with `git add -f`.
field_testing/sites/*/runs/

# Explicit site-local scratch areas. Confirmed outputs belong in analysis,
# site_inventory, or mission_plans instead.
field_testing/sites/*/scratch/
field_testing/sites/*/tmp/
```

Do not add broad ignore rules for any of these paths:

```text
field_testing/tools/
field_testing/sites/*/analysis/
field_testing/sites/*/site_inventory/
field_testing/sites/*/mission_plans/
obsidian_vault/
```

Those locations contain repository records and should remain visible in
`git status` until their contents are either committed or deliberately
reclassified.

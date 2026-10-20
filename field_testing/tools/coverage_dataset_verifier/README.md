# Coverage Dataset Verifier

`REVIEW ONLY — NOT APPROVED FOR FIELD USE`

This local browser tool implements the first approval gate in the coverage
planning workflow: identifying and verifying source files. It does not create
boundaries, obstacles, coverage sections, stripes, turns, missions, or
launchers.

## Start it on the development machine

From the repository root:

```bash
python3 field_testing/tools/coverage_dataset_verifier/app.py
```

Then open `http://127.0.0.1:8765`.

The initial project is prefilled for 62 Collins. Each source now separates:

- **Source type** — whether it is a raw recording, extracted path, derived
  review artifact, existing inventory, or supporting reference;
- **Used for** — one or more purposes: boundary, obstacle, transition,
  coordinate-frame, or supporting evidence.

Choose **View evidence** to see the file's contained lines or polygons, feature
names, and recorded lineage. Derived 62 Collins artifacts also link to their
earlier detailed interactive viewer where available. Those legacy viewers are
visual evidence only and cannot record approval.

After reviewing a file, set its **Source decision** to **Use this source**, **Do
not use**, or **Needs replacement**. Every required file must explicitly say
**Use this source** before Tab 1 can be approved.

Choose **Verify all required files** to run mechanical checks on every required
file. The individual **Verify file** button is useful after changing just one
path. Mechanical verification confirms file identity and structure; it does
not approve boundaries, obstacles, transitions, or field use.

The 62 Collins project also shows a separate **Prior approved perimeter
package** card. Use **Play mission replay** to watch the exact 3,989-waypoint
perimeter route that Al previously approved for one directly supervised,
blades-off field test. This card preserves earlier work and does not participate
in the source-approval checklist. Playing it records no verification or
approval action.

After reviewing the source evidence, enter reviewer information and approve
Tab 1 only when the listed inputs are correct. Boundary decisions will be made
in Tab 2 and obstacle decisions in Tab 3; those later tabs remain locked until
their workflows are implemented and separately reviewed.

## Saved records

The default 62 Collins output directory is:

```text
field_testing/sites/62_Collins_multi_boundary_20260915/planning/source_dataset_verification/62_COLLINS/
```

The tool writes:

- `site-project.json` — revision-numbered source configuration and verification result;
- `approval-ledger.json` — append-only events linked by SHA-256 hashes.

The records distinguish source-input approval from field validation and mission
approval. Later gates remain locked and mission generation remains disabled.

## Reuse for another yard

Choose **New blank site**. This creates an empty project with no 62 Collins
files, hashes, geometry, identifiers, or approvals. Add only the new yard's own
recordings and derived evidence. The application, schema, and review questions
are reusable; site data is not. All paths must remain inside the repository.

## Tests

```bash
python3 -m unittest discover -s field_testing/tools/coverage_dataset_verifier/tests -v
```

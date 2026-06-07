"""uavros2-asset — CLI for the asset descriptor system.

Sub-commands
------------
list      Show every primitive + UAV in the catalog.
show      Pretty-print one descriptor.
validate  Schema + cross-catalog checks. Exits non-zero on any problem.
build     Regenerate model.sdf / model.config / PX4 airframe for one UAV
          (or all of them).
diff      Like `build` but writes to a temp dir and prints `diff -u` against
          the committed files. Useful for "would this descriptor change
          anything?" workflows.
"""

from __future__ import annotations

import difflib
import pathlib
import shutil
import sys
import tempfile
from typing import List, Optional

import typer
import yaml

from .asset_builder import build_gazebo, build_px4
from .asset_spec import Catalog, load_any, validate_catalog


app = typer.Typer(
    name="uavros2-asset",
    help="uavros2 asset descriptor CLI. The descriptor under assets/ is the source of truth.",
    add_completion=False,
)

# Repo root containing assets/. Defaults to the cwd when run from the
# package source dir; override with --root.
ROOT_OPT = typer.Option(
    pathlib.Path("."),
    "--root",
    "-R",
    help="Repository root containing assets/ and models/. Defaults to cwd.",
)
TARGETS_OPT = typer.Option(
    ["gazebo", "px4"],
    "--target",
    "-t",
    help="Which generators to run. Repeat for multiple. Default: gazebo px4.",
)


def _load_catalog(root: pathlib.Path) -> Catalog:
    assets_dir = root / "assets"
    if not assets_dir.is_dir():
        typer.secho(
            f"No assets/ directory under {root}; nothing to do.", fg="red"
        )
        raise typer.Exit(2)
    return Catalog(assets_dir)


# ---------------------------------------------------------------------------
# list
# ---------------------------------------------------------------------------

@app.command("list")
def cmd_list(root: pathlib.Path = ROOT_OPT):
    """Show every primitive and assembly in the catalog."""
    cat = _load_catalog(root)
    for kind, names in cat.summary().items():
        typer.secho(f"{kind} ({len(names)}):", fg="cyan", bold=True)
        for n in names:
            typer.echo(f"  • {n}")


# ---------------------------------------------------------------------------
# show
# ---------------------------------------------------------------------------

@app.command("show")
def cmd_show(
    name: str = typer.Argument(..., help="Descriptor name (UAV/chassis/sensor/arm)."),
    root: pathlib.Path = ROOT_OPT,
):
    """Dump one descriptor as resolved YAML."""
    cat = _load_catalog(root)
    found = None
    for bucket in (cat.uavs, cat.chassis, cat.sensors, cat.mounts,
                    cat.arms, cat.airfoils):
        if name in bucket:
            found = bucket[name]
            break
    if found is None:
        typer.secho(f"No descriptor named {name!r}", fg="red")
        raise typer.Exit(2)
    typer.echo(yaml.safe_dump(found.model_dump(), sort_keys=False))


# ---------------------------------------------------------------------------
# validate
# ---------------------------------------------------------------------------

@app.command("validate")
def cmd_validate(root: pathlib.Path = ROOT_OPT):
    """Run schema + cross-catalog checks. Exit 0 if clean, 1 if problems."""
    cat = _load_catalog(root)
    problems = validate_catalog(cat)
    if not problems:
        typer.secho(
            f"✓ {len(cat.uavs)} UAVs, {len(cat.chassis)} chassis, "
            f"{len(cat.sensors)} sensors, {len(cat.arms)} arms — all valid.",
            fg="green",
        )
        raise typer.Exit(0)
    typer.secho(f"✗ {len(problems)} problem(s):", fg="red", bold=True)
    for p in problems:
        typer.echo(f"  - {p}")
    raise typer.Exit(1)


# ---------------------------------------------------------------------------
# build
# ---------------------------------------------------------------------------

def _do_build(
    catalog: Catalog,
    uav_names: List[str],
    targets: List[str],
    out_dir: pathlib.Path,
) -> List[pathlib.Path]:
    written: List[pathlib.Path] = []
    for name in uav_names:
        if name not in catalog.uavs:
            typer.secho(f"No UAV named {name!r}; skipping.", fg="red")
            continue
        uav = catalog.uavs[name]
        if "gazebo" in targets and not uav.overlays.gazebo.extra_xml is None:
            if not uav.overlays.webots.not_supported:
                pass  # placeholder; Webots stays unsupported on this code path
            written += build_gazebo(uav, catalog, out_dir)
        if "px4" in targets:
            written += build_px4(uav, catalog, out_dir)
    return written


@app.command("build")
def cmd_build(
    names: List[str] = typer.Argument(
        None,
        help="UAV descriptor names to build. Omit or pass --all to build everything.",
    ),
    all_: bool = typer.Option(False, "--all", "-a", help="Build every UAV in the catalog."),
    targets: List[str] = TARGETS_OPT,
    root: pathlib.Path = ROOT_OPT,
):
    """Regenerate Gazebo + PX4 artefacts under <root>/{models,config}/."""
    cat = _load_catalog(root)
    problems = validate_catalog(cat)
    if problems:
        typer.secho("Refusing to build — validation failed:", fg="red", bold=True)
        for p in problems:
            typer.echo(f"  - {p}")
        raise typer.Exit(1)
    if all_:
        names = sorted(cat.uavs)
    if not names:
        typer.secho("Pass a name or --all.", fg="yellow")
        raise typer.Exit(2)
    written = _do_build(cat, names, targets, root)
    typer.secho(f"✓ wrote {len(written)} files", fg="green")
    for p in written:
        typer.echo(f"  • {p.relative_to(root)}")


# ---------------------------------------------------------------------------
# diff
# ---------------------------------------------------------------------------

@app.command("diff")
def cmd_diff(
    names: List[str] = typer.Argument(
        None, help="UAV descriptor names to diff. Omit or pass --all for everything."
    ),
    all_: bool = typer.Option(False, "--all", "-a"),
    targets: List[str] = TARGETS_OPT,
    root: pathlib.Path = ROOT_OPT,
):
    """Build to a temp dir and print unified diffs vs the committed files."""
    cat = _load_catalog(root)
    if all_:
        names = sorted(cat.uavs)
    if not names:
        typer.secho("Pass a name or --all.", fg="yellow")
        raise typer.Exit(2)
    with tempfile.TemporaryDirectory(prefix="uavros2_build_") as td:
        td_path = pathlib.Path(td)
        # Shadow the directory layout so the generator writes into it.
        written = _do_build(cat, names, targets, td_path)
        any_diff = False
        for w in written:
            rel = w.relative_to(td_path)
            existing = root / rel
            new_text = w.read_text().splitlines(keepends=True)
            old_text = (
                existing.read_text().splitlines(keepends=True)
                if existing.is_file()
                else []
            )
            if new_text == old_text:
                continue
            any_diff = True
            typer.secho(f"\n--- {rel} (would change)", fg="cyan", bold=True)
            sys.stdout.writelines(
                difflib.unified_diff(
                    old_text, new_text,
                    fromfile=f"a/{rel}",
                    tofile=f"b/{rel}",
                    n=3,
                )
            )
        if not any_diff:
            typer.secho("✓ no diffs — committed files match the descriptors.", fg="green")


def main():
    app()


if __name__ == "__main__":
    main()

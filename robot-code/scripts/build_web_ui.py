Import("env")

import os
import re
import subprocess
from pathlib import Path


project_dir = Path(env.subst("$PROJECT_DIR")).resolve()
repository_dir = project_dir.parent
web_dir = repository_dir / "web-ui-react"
generated_header = project_dir / "src" / "generated" / "WebUiBundle.h"
version_source = project_dir / "src" / "WebController.cpp"
version_match = re.search(
    r'ROBOT_FIRMWARE_VERSION\[\]\s*=\s*"([^"]+)"',
    version_source.read_text(encoding="utf-8"),
)
if not version_match:
    raise RuntimeError("ROBOT_FIRMWARE_VERSION not found in WebController.cpp")
firmware_version = version_match.group(1)

env.Replace(PROGNAME=f"wrobot_firmware_{firmware_version}")


def source_files():
    paths = [
        web_dir / "src",
        web_dir / "scripts",
        web_dir / "index.html",
        web_dir / "package.json",
        web_dir / "package-lock.json",
        web_dir / "vite.config.ts",
        web_dir / "tsconfig.json",
        web_dir / "tsconfig.app.json",
        web_dir / "tsconfig.node.json",
    ]
    for path in paths:
        if path.is_file():
            yield path
        elif path.is_dir():
            yield from (item for item in path.rglob("*") if item.is_file())


def web_bundle_is_stale():
    if not generated_header.exists():
        return True
    header_time = generated_header.stat().st_mtime
    return any(path.stat().st_mtime > header_time for path in source_files())


if web_bundle_is_stale():
    npm = "npm.cmd" if os.name == "nt" else "npm"
    print("[Web UI] Sources changed; rebuilding embedded bundle...")
    result = subprocess.run(
        [npm, "run", "build:esp"],
        cwd=web_dir,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    print(result.stdout.encode("ascii", "replace").decode("ascii"))
    if result.returncode != 0:
        raise subprocess.CalledProcessError(result.returncode, result.args)
else:
    print("[Web UI] Embedded bundle is up to date.")

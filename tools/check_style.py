#!/usr/bin/env python3
"""Read-only style checks; missing tools are reported as NOT_READY."""
import argparse
import shutil
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--compile-db', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    excluded = {'build', 'install', 'log', 'generated', '_theme', '.git'}
    source_files = [
        p for p in root.rglob('*')
        if p.is_file() and not excluded.intersection(p.relative_to(root).parts)
    ]
    cpp = [p for p in source_files if p.suffix in {'.cpp', '.hpp', '.h', '.cc'}]
    py = [p for p in source_files if p.suffix == '.py']
    yaml_files = [p for p in source_files if p.suffix in {'.yaml', '.yml'}]
    xml = [p for p in source_files if p.suffix in {'.xml', '.urdf', '.xacro'}]
    failed = []
    missing = []
    checked = []

    def run(tool, arguments):
        if shutil.which(tool) is None:
            missing.append(tool)
            return
        command = [tool, *map(str, arguments)]
        print('CHECK:', ' '.join(command), flush=True)
        try:
            result = subprocess.run(command, cwd=root, timeout=120, check=False)
            if result.returncode:
                failed.append(tool)
            else:
                checked.append(tool)
        except subprocess.TimeoutExpired:
            failed.append(tool + ': timeout')

    if cpp:
        for file in cpp:
            run('clang-format', ['--dry-run', '--Werror', file])
        translations = [p for p in cpp if p.suffix in {'.cpp', '.cc'}]
        if translations:
            if args.compile_db is None:
                missing.append('C++ compile_commands.json: pass --compile-db')
            else:
                database = args.compile_db.expanduser().resolve()
                if not (database / 'compile_commands.json').is_file():
                    missing.append('compile_commands.json not found')
                else:
                    for file in translations:
                        run('clang-tidy', [file, '-p', database, '--warnings-as-errors=readability-function-size,readability-identifier-naming'])
    if py:
        run('ruff', ['check', *py])
        run('ruff', ['format', '--check', *py])
    if yaml_files:
        config = next((p / '.yamllint' for p in [root, *root.parents]
                           if (p / '.yamllint').is_file()), root / '.yamllint')
        run('yamllint', ['-c', config, *yaml_files])
        prettier = next((p / 'node_modules/.bin/prettier' for p in [root, *root.parents]
                         if (p / 'node_modules/.bin/prettier').is_file()),
                        root / 'node_modules/.bin/prettier')
        if not prettier.is_file():
            missing.append('local Prettier CLI: node_modules/.bin/prettier')
        else:
            run(str(prettier), ['--check', *yaml_files])
    for file in xml:
        run('xmllint', ['--noout', file])

    print('Successful checks:', ', '.join(sorted(set(checked))) or 'none')
    if failed:
        print('CHECK_FAILED:', ', '.join(sorted(set(failed))))
    if missing:
        print('NOT_READY:', ', '.join(sorted(set(missing))))
    print('These checks do not run or verify ROS behavior.')
    if failed:
        return 1
    if missing:
        return 2
    if not checked:
        print('NO_CHECKS: no applicable source files')
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

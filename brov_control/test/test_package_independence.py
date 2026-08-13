"""Prevent reintroduction of the former monorepo path dependency."""

import ast
from pathlib import Path


def test_runtime_modules_do_not_import_deploy_or_mutate_sys_path():
    package_dir = Path(__file__).parents[1] / "brov_control"
    for source_path in package_dir.glob("*.py"):
        source = source_path.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(source_path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                assert all(name.name != "deploy" for name in node.names)
            elif isinstance(node, ast.ImportFrom):
                assert node.module != "deploy"
                assert not (node.module or "").startswith("deploy.")
        assert "sys.path" not in source


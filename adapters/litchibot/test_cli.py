import ast
from pathlib import Path
from unittest.mock import Mock

import pytest


def test_unified_dry_run_precedes_all_arm_and_output_initialization(monkeypatch):
    from teleop_runtime import cli
    from adapters.litchibot import cli as litchi_cli
    run=Mock(return_value=0)
    monkeypatch.setattr(litchi_cli,'run_dry_run',run)
    monkeypatch.setattr(cli,'DualFr3HardwareTeleop',Mock(side_effect=AssertionError('FR3 initialized')))
    monkeypatch.setattr(cli,'load_config',Mock(side_effect=AssertionError('arm config loaded')))
    monkeypatch.setattr('sys.argv',['teleop','--config','missing.yaml','--arm-source','gello',
                                  '--hand-source','litchibot','--dry-run'])
    with pytest.raises(SystemExit) as done:cli.main()
    assert done.value.code==0
    run.assert_called_once()


def test_dry_run_cannot_be_ambiguous_with_real_output(monkeypatch):
    from teleop_runtime import cli
    monkeypatch.setattr('sys.argv',['teleop','--config','missing.yaml','--arm-source','gello',
        '--hand-source','litchibot','--dry-run','--enable-hand-output'])
    with pytest.raises(SystemExit) as done:cli.main()
    assert done.value.code==2


def test_manus_branch_in_unified_cli_is_byte_equivalent_to_git_head():
    import subprocess
    root=Path(__file__).resolve().parents[2]
    original=subprocess.check_output(['git','show','HEAD:teleop_runtime/cli.py'],cwd=root,text=True)
    updated=(root/'teleop_runtime/cli.py').read_text()
    def branch(source):
        for node in ast.walk(ast.parse(source)):
            if isinstance(node,ast.If) and ast.unparse(node.test)=="args.hand_source == 'manus'":
                return [ast.dump(n,include_attributes=False) for n in node.body]
        raise AssertionError('Manus branch missing')
    assert branch(original)==branch(updated)

"""GUI regression tests must never alter the operator's real transfer policy."""
import pytest


@pytest.fixture(autouse=True)
def isolate_transfer_preferences(tmp_path,monkeypatch):
    monkeypatch.setenv('COLLECTION_TRANSFER_CONFIG',str(tmp_path/'transfer-config.json'))
    monkeypatch.setenv('COLLECTION_TRANSFER_STATE',str(tmp_path/'transfer-state'))
    monkeypatch.setenv('OPERATOR_TASKS_FILE',str(tmp_path/'tasks.json'))

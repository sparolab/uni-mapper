import open_lmm


def run(config_directory: str) -> open_lmm.RuntimeSnapshot:
    with open_lmm.Runtime() as runtime:
        runtime.open(config_directory, label="python-example")
        runtime.run_all().wait()
        return runtime.snapshot()

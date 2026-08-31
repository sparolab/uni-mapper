import open_lmm


def cancel(config_directory: str) -> None:
    with open_lmm.Runtime() as runtime:
        runtime.open(config_directory, label="cancel-example")
        job = runtime.run_all()
        job.cancel()
        try:
            job.wait()
        except open_lmm.OpenLMMCancelledError:
            pass

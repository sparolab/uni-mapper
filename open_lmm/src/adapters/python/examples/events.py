import open_lmm


def observe(config_directory: str) -> None:
    with open_lmm.Runtime() as runtime:
        runtime.open(config_directory, label="events-example")
        with runtime.subscribe_events(
            lambda event: print(event.sequence, event.type.name, event.message)
        ):
            runtime.run_all().wait()

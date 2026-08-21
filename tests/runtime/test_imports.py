def test_production_composition_and_comm_import_without_cycles():
    import src.comm.mavlink_node  # noqa: F401
    import src.runtime.composition  # noqa: F401

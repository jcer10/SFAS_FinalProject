def extract_by_name(model_states, starts_with, reversed=False):
    # keys = model_states.__slots__
    keys = ['name', 'pose', 'twist']
    zipped_model_states = zip(
        *[getattr(model_states, key) for key in keys]
    )

    zipped_cube_states = list(filter(
        lambda zipped_model_state: zipped_model_state[0].startswith(starts_with),
        zipped_model_states))

    #return [{ k:v for (k, v) in zip(keys, zipped_cube_state)} for zipped_cube_state in zipped_cube_states]
    stepper = -1 if reversed else 1
    return zipped_cube_states[::stepper]
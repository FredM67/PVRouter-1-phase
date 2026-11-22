# Bug 6: every load remote and no override pin, so the router uses no pin at all.
loads 3 'Load::remote(1), Load::remote(1), Load::remote(2)'
replace 'OVERRIDE_PIN_PRESENT{ true }' 'OVERRIDE_PIN_PRESENT{ false }'

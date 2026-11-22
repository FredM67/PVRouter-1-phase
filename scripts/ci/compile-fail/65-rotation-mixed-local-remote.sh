# Two local loads and a remote one in one group.
replace 'PRIORITY_ROTATION{ RotationModes::OFF }' 'PRIORITY_ROTATION{ RotationModes::AUTO }'
loads 3 'Load::local(5), Load::local(6), Load::remote(1)'
replace 'rotationGroups{};' 'rotationGroups{ { ALL_LOADS() } };'
expect='A rotation group mixes local and remote loads, or loads of two remote units'

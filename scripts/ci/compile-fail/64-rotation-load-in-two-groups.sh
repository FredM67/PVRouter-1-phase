# Load 1 in two groups.
loads 3 'Load::local(5), Load::local(6), Load::local(7)'
replace 'PRIORITY_ROTATION{ RotationModes::OFF }' 'PRIORITY_ROTATION{ RotationModes::AUTO }'
replace 'rotationGroups{};' 'rotationGroups{ { LOAD(0), LOAD(1) }, { LOAD(1), LOAD(2) } };'
expect='A load is listed twice in rotationGroups'

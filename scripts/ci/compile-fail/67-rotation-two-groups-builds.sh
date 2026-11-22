# Two groups: two local loads, and two loads on remote unit 1.
replace 'PRIORITY_ROTATION{ RotationModes::OFF }' 'PRIORITY_ROTATION{ RotationModes::AUTO }'
loads 4 'Load::local(5), Load::local(6), Load::remote(1), Load::remote(1)'
replace 'rotationGroups{};' 'rotationGroups{ { LOAD(0), LOAD(1) }, { LOAD(2), LOAD(3) } };'
expect=''

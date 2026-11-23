# Groups given while rotation is OFF.
loads 3 'Load::local(5), Load::local(6), Load::local(7)'
replace 'rotationGroups{};' 'rotationGroups{ { ALL_LOCAL_LOADS() } };'
expect='rotationGroups is set but priority rotation is OFF'

# alpha too small for the resolution of its fixed point.
replace 'lpf_gainNewBoard{ 0 }' 'lpf_gainNewBoard{ 9 }'
replace 'alphaNewBoard{ 0.002 }' 'alphaNewBoard{ 0.00001 }'
expect='Wrong CT filter settings'

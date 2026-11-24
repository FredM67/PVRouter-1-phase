# The CT filter: alpha from 0.00005 to below 0.0625 (fixed point, ct_filter.h).
replace 'lpf_gainNewBoard{ 0 }' 'lpf_gainNewBoard{ 9 }'
replace 'alphaNewBoard{ 0.002 }' 'alphaNewBoard{ 0.1 }'
expect='Wrong CT filter settings'

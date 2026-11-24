# The gain of the CT filter must stay below 64.
replace 'lpf_gainNewBoard{ 0 }' 'lpf_gainNewBoard{ 64 }'
expect='Wrong CT filter settings'

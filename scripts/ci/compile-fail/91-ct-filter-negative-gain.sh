# A negative gain is no filter.
replace 'lpf_gainNewBoard{ 0 }' 'lpf_gainNewBoard{ -1 }'
expect='Wrong CT filter settings'

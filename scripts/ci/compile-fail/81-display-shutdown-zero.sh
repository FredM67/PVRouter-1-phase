# A shutdown after 0 hours would reset the diverted energy at once.
replace 'DISPLAY_SHUTDOWN_IN_HOURS{ 8 }' 'DISPLAY_SHUTDOWN_IN_HOURS{ 0 }'
expect='DISPLAY_SHUTDOWN_IN_HOURS must be greater than 0'

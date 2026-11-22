# With CT2, an absurd rating is still refused.
replace 'f_ctRatingDiverted{ 50.0F }' 'f_ctRatingDiverted{ 100000.0F }'
expect='Wrong CT2 power calibration'

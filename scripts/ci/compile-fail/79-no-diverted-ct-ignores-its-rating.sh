# Without CT2, its rating is never used: an absurd one does not stop the build.
replace 'DIVERTED_CT_PRESENT{ true }' 'DIVERTED_CT_PRESENT{ false }'
replace 'f_ctRatingDiverted{ 50.0F }' 'f_ctRatingDiverted{ 100000.0F }'
expect=''

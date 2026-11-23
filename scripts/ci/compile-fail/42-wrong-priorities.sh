# The same priority twice.
loads 3 'Load::local(5), Load::local(6), Load::local(7)' '0, 1, 1'
expect='Load Priorities wrong'

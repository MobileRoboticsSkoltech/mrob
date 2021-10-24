import mrob


print('from mrob import mrob\n')

for module in dir(mrob):
    n = len(module)-1
    if not (module[:2] == '__' and module[n:n-2:-1] == '__') and module.count('.') == 0:
        print(module + ' = mrob.' + module)

print('\ndel(mrob)')

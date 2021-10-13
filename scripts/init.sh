#!/bin/bash

IFS=$'\n' MODULES=$(python3 -c \
"import mrob
for x in dir(mrob):
     if not (len(x) > 2 and x[0] == '_' and x[1] == '_'):
         print(x)")

echo -e "from mrob import mrob\n" >> __init__.py
for module in $MODULES
do
    if ! [[ $module == *"."* ]]; then
        echo $module = mrob.$module >> __init__.py
    fi
done
echo -e "\ndel(mrob)" >> __init__.py

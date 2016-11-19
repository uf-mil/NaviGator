# flake8: noqa
import os

for module in os.listdir(os.path.dirname(__file__)):
<<<<<<< HEAD
    if module == '__init__.py' or module[-3:] != '.py' or module[-1] == '~':
=======
<<<<<<< HEAD
    if module == '__init__.py' or module[-3:] != '.py' and module[-1] != '~':
=======
    if module == '__init__.py' or module[-3:] != '.py' or module[-1] == '~':
>>>>>>> be0a9db... DETECT DELIVER: fix circle search, pr comments
>>>>>>> DETECT DELIVER: fix circle search, pr comments
        continue
    __import__(module[:-3], locals(), globals())

del module
del os

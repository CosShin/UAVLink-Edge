from setuptools import setup
from setuptools.command.develop import develop as _develop
from setuptools.command.install import install as _install

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from _apt import apt_install


class Install(_install):
    def run(self):
        apt_install()
        super().run()


class Develop(_develop):
    def run(self):
        apt_install()
        super().run()


setup(
    cmdclass={"install": Install, "develop": Develop},
)

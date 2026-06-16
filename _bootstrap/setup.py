from setuptools import setup
from setuptools.command.develop import develop as _develop
from setuptools.command.install import install as _install

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

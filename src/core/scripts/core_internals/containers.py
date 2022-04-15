
from .parameters import ParametersConfigurator

from dependency_injector import containers, providers


class GlobalContainer(containers.DeclarativeContainer):

    parametes_updater = providers.Singleton(
        ParametersConfigurator
    )

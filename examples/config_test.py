
import os
import yaml

_defaultSettingsFile = "settings.yaml"

class Settings:
    '''
    '''

    def __init__(self, file=None):

        if file is None:
            file = _defaultSettingsFile
        else:
            pass

        self.path = os.path.join(os.getcwd(), file)

        # Open settings file
        with open(self.path, "r") as settingsFile:
            # self.settings = yaml.load(settingsFile)
            settings = yaml.safe_load(os.path.expandvars(settingsFile.read()))


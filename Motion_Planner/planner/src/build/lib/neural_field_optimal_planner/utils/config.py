from pytorch_lightning.utilities import AttributeDict


class Config(object):
    def __init__(self, dictionary):
        self._dictionary = dictionary

    @classmethod
    def from_dict(cls, dictionary):
        return cls(dictionary)

    def as_attribute_dict(self):
        return self._make_attribute_dict(self._dictionary)

    @staticmethod
    def _make_attribute_dict(dictionary):
        if not isinstance(dictionary, dict) or not isinstance(dictionary, AttributeDict):
            return dictionary
        new_dictionary = {}
        for key, value in dictionary.items():
            new_dictionary[key] = Config._make_attribute_dict(value)
        return AttributeDict(**new_dictionary)

    def update(self, dictionary):
        self.update_dictionary(self._dictionary, dictionary)

    @staticmethod
    def update_dictionary(old_dictionary, new_dictionary):
        for key, value in new_dictionary.items():
            if not isinstance(value, dict) and not isinstance(value, AttributeDict):
                old_dictionary[key] = value
            elif key not in old_dictionary.keys():
                old_dictionary[key] = value
            elif not isinstance(old_dictionary[key], dict) and not isinstance(old_dictionary[key], AttributeDict):
                old_dictionary[key] = value
            else:
                Config.update_dictionary(old_dictionary[key], value)
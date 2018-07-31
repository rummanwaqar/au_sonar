import yaml
import os


class PreprocessorCfg(object):
    def __init__(self):
        pass

    @classmethod
    def from_file(cls, file_name):
        pass

    def verify(self):
        pass

if __name__ == '__main__':
    url='../cfg/preprocessor.cfg'
    if os.path.exists(url):
        params = yaml.load(open(url))
        for param, value in params.iteritems():
                print(param, value)

    else:
        print("ERRROROROR")
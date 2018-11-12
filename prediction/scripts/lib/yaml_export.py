#!/usr/bin/env python

from keras.models import load_model
import yaml
import base64

#def get_bootstrap_normalization():
#    out_min=[-0.04123226483869708, -0.031217403199005074, -0.22898143957295725]
#    out_max=[0.03178563295947549, 0.029346353059715446, 0.26358129169260636]
#    center_out = [0.5 * (mi + ma) for mi, ma in zip(out_min, out_max)]
#    scale_out = [ 1 / (ma - mi) for mi, ma in zip(out_min, out_max)]
#
#    normalization = {}
#    normalization["input"] = { }
#    normalization["output"] = { }
#    normalization["input"]["center"] = center_in[0].tolist()
#    normalization["output"]["center"] = center_out[0].tolist()
#    normalization["input"]["scale"] = scale_in[0].tolist()
#    normalization["output"]["scale"] = scale_out[0].tolist()


def load_keras_model(model_file):
    return load_model(model_file)

def dump_model_to_yaml(model, filename, normalization = None):
    data = {}
    data["layers"] = yaml.load(model.to_yaml())["config"]
    def encodeweights(l):
        if len(l.shape) == 1:
            return base64.b64encode(l.tobytes())
        if len(l.shape) > 1:
            return [encodeweights(a) for a in l]

    data["weights"] = [[encodeweights(weightarray) for weightarray in layer.get_weights()] for layer in model.layers]

    #if normalization is None:
    #    normalization = get_bootstrap_normalization()
    #
    #data["normalization"] = normalization
    #data["normalization"]["input"] = { }
    #data["normalization"]["output"] = { }
    #data["normalization"]["input"]["center"] = center_in[0].tolist()
    #data["normalization"]["output"]["center"] = center_out[0].tolist()
    #data["normalization"]["input"]["scale"] = scale_in[0].tolist()
    #data["normalization"]["output"]["scale"] = scale_out[0].tolist()
    yaml.dump(data, open(filename, "w"))

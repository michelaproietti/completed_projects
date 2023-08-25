#!/usr/bin/env python
# coding: utf-8
# %%
from tensorflow.keras import Model
#from tensorflow.keras.initializers import he_uniform
from tensorflow.keras.layers import Conv2D, BatchNormalization, ReLU, Activation
from tensorflow.keras.activations import linear
from tensorflow.keras import mixed_precision


# %%
# mixed precision

policy = mixed_precision.Policy('mixed_float16')
mixed_precision.set_global_policy(policy)
print('Compute dtype: %s' % policy.compute_dtype)
print('Variable dtype: %s' % policy.variable_dtype)


# %%
class ConvBlock(Model):
    def __init__(self, n_filters, kernel_size, activation=True):
        super(ConvBlock, self).__init__()
        self.n_filters = n_filters
        self.kernel_size = kernel_size
        self.activation = activation
        
        self.conv = Conv2D(filters=self.n_filters, kernel_size=self.kernel_size, padding='same')#, kernel_initializer=he_uniform())
        self.batch_norm = BatchNormalization()
        
        if self.activation:
            self.relu = ReLU()
        
    def call(self, x):
        x = self.conv(x)
        x = self.batch_norm(x)
        if self.activation:
            x = self.relu(x)
        return x


# %%
class DnCNN(Model):
    def __init__(self, n_filters:int, n_output_channels:int, kernel_size:int, hidden_layers:int, internal_res=False):
        super(DnCNN, self).__init__()
        self.n_filters = n_filters
        self.kernel_size = kernel_size
        self.n_output_channels = n_output_channels
        self.hidden_layers = hidden_layers
        self.internal_res = internal_res
        
        self.first_layer = Conv2D(filters=self.n_filters, kernel_size=self.kernel_size,
                                  padding='same', activation='relu')#, kernel_initializer=he_uniform())
        
        #self.conv_layers = [ConvBlock(64, 3) for i in range(hidden_layers)]
        
        self.res_blocks = []
        for i in range(0, hidden_layers, 2):
            self.res_blocks.append(ConvBlock(64, 3))
            self.res_blocks.append(ConvBlock(64, 3, activation=False))
            
        self.relu = ReLU()
        
        self.output_layer = Conv2D(filters=self.n_output_channels, kernel_size=self.kernel_size,
                             padding='same', dtype='float32')#, kernel_initializer=he_uniform())
        
        self.linear = Activation('linear', dtype='float32')
        
    def call(self, x):
            out = self.first_layer(x)
            
            '''
            for conv in self.conv_layers:
                out = conv(out)
            '''
            
            if self.internal_res:
                out_res = out
            
            for index in range(self.hidden_layers):
                out = self.res_blocks[index](out)
                
                if self.internal_res and index % 2 == 1:
                    out = out + out_res
                    out = self.relu(out)
                    out_res = out
            
            out = self.linear(out)
            out = self.output_layer(out)
            
            x = self.linear(x)
            
            out = x - out
            
            return out


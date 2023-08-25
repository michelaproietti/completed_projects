from tensorflow.keras import Model
from tensorflow.keras.initializers import he_uniform, random_normal
from tensorflow.keras.layers import Conv2D, BatchNormalization, ReLU, Activation, LeakyReLU
from tensorflow.keras.activations import linear
from tensorflow.keras import mixed_precision

# mixed precision

policy = mixed_precision.Policy('mixed_float16')
mixed_precision.set_global_policy(policy)
print('Compute dtype: %s' % policy.compute_dtype)
print('Variable dtype: %s' % policy.variable_dtype)

class ConvBlock(Model):
    def __init__(self, n_filters, kernel_size):
        super(ConvBlock, self).__init__()
        self.n_filters = n_filters
        self.kernel_size = kernel_size
        
        self.conv = Conv2D(filters=self.n_filters, kernel_size=self.kernel_size, padding='same', kernel_initializer=he_uniform())
        self.batch_norm = BatchNormalization()
        self.relu = ReLU()
        
    def call(self, x):
        x = self.conv(x)
        x = self.batch_norm(x)
        x = self.relu(x)
        return x

class DnCNN(Model):
    def __init__(self, n_filters:int, n_output_channels:int, kernel_size:int, hidden_layers:int):
        super(DnCNN, self).__init__()
        self.n_filters = n_filters
        self.kernel_size = kernel_size
        self.n_output_channels = n_output_channels
        
        self.first_layer = Conv2D(filters=self.n_filters, kernel_size=self.kernel_size,
                                  padding='same', kernel_initializer=he_uniform())
        self.relu = ReLU()
        
        self.conv_layers = [ConvBlock(64, 3) for i in range(hidden_layers)]
        
        self.output_layer = Conv2D(filters=self.n_output_channels, kernel_size=self.kernel_size,
                             padding='same', dtype='float32', kernel_initializer=he_uniform())
        
        self.linear = Activation(activation='linear', dtype='float32')
        
    def call(self, x):
        out = self.first_layer(x)
        out = self.relu(out)

        for conv in self.conv_layers:
            out = conv(out)

        x = self.linear(x)
        out = self.output_layer(out)

        out = x - out

        return out

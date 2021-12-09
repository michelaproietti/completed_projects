import tensorflow as tf
from tensorflow.keras import Model, Sequential
from tensorflow.keras.layers import Input, Dense
from tensorflow.keras.optimizers import Adam
from datetime import datetime
import numpy as np

# NN-based state-value function
class NNValueFunction(object):
    def __init__(self, obs_dim, hid1_size):
        self.replay_buffer_x = None
        self.replay_buffer_y = None
        self.obs_dim = obs_dim # dimensions of the observation space
        self.hid1_size = hid1_size # hid1_size: size of first hidden layer, multiplier of obs_dim
        self.epochs = 10
        self.lr = None  # learning rate set in _build_model()
        self.valueModel = self._build_model()
    
    # Construct Tensorflow graph, including loss function, init op and train opobs
    def _build_model(self):
        inputLayer = Input(shape=(self.obs_dim,), dtype='float32')
        # layer size: hid1 is obs_dim*10; hid2 is geometric mean; hid3 is 5 (chosen empirically)
        hid1_units = self.obs_dim * self.hid1_size
        hid3_units = 5
        hid2_units = int(np.sqrt(hid1_units * hid3_units))
        # heuristic to set learning rate based on NN size
        self.lr = 1e-2 / np.sqrt(hid2_units)  # 1e-2 empirically determined
        print('Value Params -- h1: {}, h2: {}, h3: {}, lr: {:.3g}'
              .format(hid1_units, hid2_units, hid3_units, self.lr))
        # defining the model
        def model_sequential():
            model = Sequential()
            model.add(Dense(hid1_units, activation=tf.nn.leaky_relu))
            model.add(Dense(hid2_units, activation=tf.nn.leaky_relu))
            model.add(Dense(hid3_units, activation=tf.nn.leaky_relu))
            model.add(Dense(1))
            return model 
        valueModel = model_sequential()
        valueModel.compile(loss="mse", optimizer=Adam(self.lr))
        return valueModel
    
    # Fit model to current data batch + previous data batch
    def fit(self, x, y, logger, episode, name):
        """ Args:
            x: features
            y: target
            logger: logger to save training loss and % explained variance
        """
        num_batches = max(x.shape[0] // 256, 1) # create batches with 256 elements or 1 single batch if the number of elements is lower
        batch_size = x.shape[0] // num_batches # compute the batch_size by dividing by the number of batches
        y_hat = self.valueModel.predict(x)  # compute estimated value before update
        old_exp_var = 1 - np.var(y - y_hat)/np.var(y) # check explained variance prior to update
        if self.replay_buffer_x is None:
            x_train, y_train = x, y
        else:
            x_train = np.concatenate([x, self.replay_buffer_x])
            y_train = np.concatenate([y, self.replay_buffer_y])
        # Each time in replay_buffer_* there are the inputs of the previous execution
        self.replay_buffer_x = x
        self.replay_buffer_y = y
        self.valueModel.fit(x_train, y_train, epochs=self.epochs, batch_size=batch_size, shuffle=True, verbose=0)
        y_hat = self.valueModel.predict(x)
        loss = np.mean(np.square(y_hat - y))
        exp_var = 1 - np.var(y - y_hat) / np.var(y)  # explained variance after update diagnose overfitting of val func
        # Saving the files log
        logger.log({'ValFuncLoss': loss,
                    'ExplainedVarNew': exp_var,
                    'ExplainedVarOld': old_exp_var})
        # Saving the weights log
        self.valueModel.save_weights(f"{name}/{episode}.ckpt") # save weights
    
    # Predict method
    def predict(self, x):
        return self.valueModel.predict(x)

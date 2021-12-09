import tensorflow as tf
import tensorflow.keras.backend as K
from tensorflow.keras import Model, Sequential
from tensorflow.keras.layers import Dense, Layer
from tensorflow.keras.optimizers import Adam
from utils import *
import numpy as np

# Defining the model of the policy network
def nn_policy_model(input_shape, output_shape, obs_dim, act_dim, hid1_size):
    hid1_units = obs_dim * hid1_size # size of input layer depends on observation space size
    hid3_units = act_dim * 4  # 4 empirically determined
    # size of the second layer based on the number of nodes of the neighbouring layers
    hid2_units = int(np.sqrt(hid1_units * hid3_units))
    model = Sequential() #  activation function leaky relu (experimentally determined)
    model.add(Dense(hid1_units, input_shape=input_shape, activation=tf.nn.leaky_relu, dtype=tf.float64))
    model.add(Dense(hid2_units, activation=tf.nn.leaky_relu, dtype=tf.float64))
    model.add(Dense(output_shape, dtype=tf.float64))
    print('Policy Params -- h1: {}, h2: {}'.format(hid1_units, hid2_units))
    return model

class Policy(object):
    def __init__(self, obs_dim, act_dim, hid1_size):
        self.delta = 0.01 # hyperparameter used in the trust region costraint
        self.epochs = 20 # number of epochs to optimize the surrogate loss w.r.t. theta
        self.policy = nn_policy_model((18,), 6, obs_dim, act_dim, hid1_size)
        self.obs_dim = obs_dim # obs_dim: size of the observation space (int)
        self.act_dim = act_dim # act_dim: size of the action space (int)
        self.epsilon = 0.4 # used in order to be epsilon-greedy while sampling the actions
        self.cg_iters = 10 # number of iterations of conjugate gradient to perform
        self.cg_damping = 0.01 # used to adjusts hessian-vector product calculation
        self.residual_tol = 1e-5 # it is used in conjugate gradient method to check whether the 
                                 # residual is small enough to exit the loop
        self.backtrack_coeff = 0.6 # used to control backtracking in line search
        self.backtrack_iters = 10 # maximum number of backtracking iterations in line search to tune 
                                  # step sizes of policy updates
        self.epsilon_decay = lambda x: x - 8e-3 # experimentally determined
    
    # Draw sample from policy with normal distribution
    def sample(self, observes, last_action, env):
        logits = self.policy(observes)
        action_prob = tf.nn.softmax(logits).numpy().ravel() # 1D array containing action probabilities
        # we pick an action with a probability given by the corresponding entry in action_prob
        action = np.argmax(action_prob)
        # epsilon greedy -> we perform a random action with probability epsilon
        if np.random.uniform(0,1) < self.epsilon:
            action = np.random.randint(0,env.action_space.shape[0])
        self.last_action = action
        return action, action_prob
    
    # Update policy based on observations, actions and advantages
    def update(self, observes, actions, advantages, logger, entropy, env):
        def surrogate_loss(theta=None):
            logits = self.policy(observes)
            action_prob = tf.nn.softmax(logits)
            action_prob = tf.reduce_sum(actions_one_hot * action_prob, axis=1)
            old_logits = self.policy(observes)    
            old_action_prob = tf.nn.softmax(old_logits)
            old_action_prob = tf.reduce_sum(actions_one_hot * old_action_prob, axis=1).numpy() + 1e-8
            prob_ratio = action_prob / old_action_prob # pi(a|s) / pi_old(a|s)
            loss = tf.reduce_mean(prob_ratio * advantages)
            return loss

        # This function returns the mean kl divergence between the old and the new outputs of the 
        # policy network, that represent a probability distribution over the actions
        def kl_fn(theta=None):
            model = self.policy
            logits = model(observes)
            action_prob = tf.nn.softmax(logits).numpy() + 1e-8
            old_logits = self.policy(observes)
            old_action_prob = tf.nn.softmax(old_logits)
            return tf.reduce_mean(tf.reduce_sum(old_action_prob * tf.math.log(old_action_prob / action_prob), axis=1))

        # This function computes the Hessian-vector product. Instead of computing the Hessian matrix, 
        # that is very expensive, we used the direct method
        def hessian_vector_product(p):
            def hvp_fn(): 
                kl_grad_vector = flatgrad(kl_fn, self.policy.trainable_variables)
                grad_vector_product = tf.reduce_sum(kl_grad_vector * p)
                return grad_vector_product

            fisher_vector_product = flatgrad(hvp_fn, self.policy.trainable_variables).numpy()
            return fisher_vector_product + (self.cg_damping * p) 
        
        # This function implements the conjugate gradient method with backtracking line search:
        # it determines the next direction along which to move in order to minimize the error function
        # and the step size, represented by the alpha value
        def conjugate_grad(Ax, b):
            """
            Conjugate gradient algorithm
            (see https://en.wikipedia.org/wiki/Conjugate_gradient_method)
            """
            x = np.zeros_like(b)
            r = b.copy() # Note: should be 'b - Ax(x)', but for x=0, Ax(x)=0
            p = r.copy()
            old_p = p.copy()
            r_dot_old = np.dot(r,r)
            for _ in range(self.cg_iters):
                z = Ax(p)
                alpha = r_dot_old / (np.dot(p, z) + 1e-8) # 1e-8 is used to handle 0 division
                old_x = x
                x += alpha * p
                r -= alpha * z
                r_dot_new = np.dot(r,r)
                beta = r_dot_new / (r_dot_old + 1e-8) # 1e-8 is used to handle 0 division
                r_dot_old = r_dot_new
                if r_dot_old < self.residual_tol: # exit the loop is the residuals is small enough
                    break
                old_p = p.copy()
                p = r + beta * p # compute next direction 
                if np.isnan(x).any():
                    print("x is nan")
                    print("z", np.isnan(z))
                    print("old_x", np.isnan(old_x))
                    print("kl_fn", np.isnan(kl_fn()))
            return x

        # This function returns the new parameters of the policy network that we obtain after 
        # performing conjugate gradient and line search. If the parameters that we get are
        # such that the kl divergence is greater than delta we don't perform the update.
        def linesearch(x, fullstep):
            fval = surrogate_loss(x)
            for (_n_backtracks, stepfrac) in enumerate(self.backtrack_coeff**np.arange(self.backtrack_iters)):
                xnew = x + stepfrac * fullstep
                newfval = surrogate_loss(xnew)
                kl_div = kl_fn(xnew)
                if np.isnan(kl_div):
                    print("kl is nan")
                    print("xnew", np.isnan(xnew))
                    print("x", np.isnan(x))
                    print("stepfrac", np.isnan(stepfrac))
                    print("fullstep",  np.isnan(fullstep))
                if kl_div <= self.delta and newfval >= 0:
                    print("Linesearch worked at ", _n_backtracks)
                    return xnew
                if _n_backtracks == self.backtrack_iters - 1:
                    print("Linesearch failed.", kl_div, newfval)
            return x
        
        # Computes the gradient of the surrogate loss w.r.t. the list of parameters and flattens it to have a 1-D vector 
        def flatgrad(loss_fn, var_list):
            with tf.GradientTape() as t:
                loss = loss_fn()
            grads = t.gradient(loss, var_list, unconnected_gradients=tf.UnconnectedGradients.ZERO)
            return tf.concat([tf.reshape(g, [-1]) for g in grads], axis=0)

        # This function assigns to the trainable parameters the updated values we compute with conjugate gradient and line search
        def assign_vars(model, theta):
                shapes = [v.shape.as_list() for v in model.trainable_variables]
                size_theta = np.sum([np.prod(shape) for shape in shapes])
                start = 0
                for i, shape in enumerate(shapes):
                    size = np.prod(shape)
                    param = tf.reshape(theta[start:start + size], shape)
                    model.trainable_variables[i].assign(param)
                    start += size
                assert start == size_theta, "messy shapes"
             
        # It returns a tensor with all the trainable variables
        def flatvars(model):
            return tf.concat([tf.reshape(v, [-1]) for v in model.trainable_variables], axis=0)

                
        actions_one_hot = tf.one_hot(actions, env.action_space.shape[0], dtype="float64") # one-hot vector with size=dimension of the action space
        policy_loss = surrogate_loss()
        policy_gradient = flatgrad(surrogate_loss, self.policy.trainable_variables).numpy()

        step_direction = conjugate_grad(hessian_vector_product, policy_gradient)
	
		# In the following three lines we compute the update for the parameters according to equation 11 in the report
        shs = .5 * step_direction.dot(hessian_vector_product(step_direction).T)
        lm = np.sqrt(shs / self.delta) + 1e-8
        fullstep = step_direction / lm
        
        if np.isnan(fullstep).any():
            print("fullstep is nan")
            print("lm", lm)
            print("step_direction", step_direction)
            print("policy_gradient", policy_gradient)

        oldtheta = flatvars(self.policy).numpy()
        theta = linesearch(oldtheta, fullstep)

        if np.isnan(theta).any():
            print("NaN detected. Skipping update...")
        else:
            assign_vars(self.policy, theta) # update the trainable parameters

        kl = kl_fn(oldtheta) # compute new kl divergence
        
        self.epsilon = max(0, self.epsilon_decay(self.epsilon)) # apply the update on the epsilon coefficient

        # Adding new informations to the log
        logger.log({'PolicyLoss': policy_loss,
                    'PolicyEntropy': entropy,
                    'KL': kl })


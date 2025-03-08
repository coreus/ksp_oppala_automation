
import tensorflow as tf
from tensorflow import keras
from keras import layers

import pandas as pd
import numpy as np


df = pd.read_csv("landing.csv")
#(df['front'] / 90).to_csv('y.csv',index=False)
df['front'] = df['front'] / 90
df['back'] = df['back'] / 90
df['pitch'] = df['pitch'] / 180
df['delta_pitch'] = df['delta_pitch'] / 90
#df[['back','pitch','delta_pitch','atmo_density']].to_csv('x.csv',index=False)


inputs = keras.Input(shape=(4,))
x = layers.Normalization(input_shape=(4,), axis=1)(inputs)
x = layers.Dense(30, activation="relu")(x)
x = layers.Dense(30, activation="relu")(x)
x = layers.Dense(1,activation="linear")(x)
model = keras.Model(inputs=inputs, outputs=x, name="landing_model")
model.summary()
model.compile(loss="mse")
model.fit(df[['back','pitch','delta_pitch','atmo_density']].to_numpy(), df['front'].to_numpy(),batch_size=256,epochs=400)
tf.keras.models.save_model(model,"landing.model")
dfo = pd.read_csv("x.csv")
print(model.predict(dfo[['back','pitch','delta_pitch','atmo_density']].to_numpy()))

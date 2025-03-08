
import tensorflow as tf
from tensorflow import keras
from keras import layers
import pandas as pd
from keras.optimizers import Adam


df = pd.read_csv("landing.csv")

df['front'] = df['front'] / 90
df['back'] = df['back'] / 90
df['pitch'] = df['pitch'] / 180
df['h_speed'] = df['h_speed'] / 150
df['v_speed'] = df['v_speed'] / 150
df['delta_pitch'] = df['delta_pitch'] / 90
(df[['back','front']] / 90).to_csv('y.csv',index=False)
df[['pitch','delta_pitch','atmo_density','h_speed','v_speed']].to_csv('x.csv',index=False)


inputs = keras.Input(shape=(5,))
x = layers.Normalization(input_shape=(5,), axis=1)(inputs)
x = layers.Dense(60, activation="relu")(x)
x = layers.Dense(60, activation="relu")(x)
x = layers.Dense(2,activation="linear")(x)
model = keras.Model(inputs=inputs, outputs=x, name="landing_model")
model.summary()
model.compile(loss="mse", optimizer=Adam(learning_rate=3e-4))
model.fit(df[['pitch','delta_pitch','atmo_density','h_speed','v_speed']].to_numpy(), df[['back','front']].to_numpy(),batch_size=1024,epochs=200)
tf.keras.models.save_model(model,"landing.v3.model")
dfo = pd.read_csv("x.csv")
print(model.predict(dfo[['pitch','delta_pitch','atmo_density','h_speed','v_speed']].to_numpy()))

import streamlit as st
import pandas as pd
import numpy as np
import pydeck as pdk
from datetime import datetime, timedelta

st.title("My GPS Data")

# Initialize connection.
conn = st.connection("postgresql", type="sql")

# Perform query.
df = conn.query('SELECT * FROM GPS;', ttl="10m")

# Convert timestamp column to datetime
df['timestamp'] = pd.to_datetime(df['timestamp'])

# Convert datetime to timestamp for the slider (including microseconds)
min_timestamp = df['timestamp'].min()
max_timestamp = df['timestamp'].max()

# print(df.head(20))

print(f"min_timestamp: {min_timestamp}")
print(f"max_timestamp: {max_timestamp}")

# Create a slider for selecting the time range
start_timestamp, end_timestamp = st.slider(
    "Select a range of timestamps",
    min_value=min_timestamp.to_pydatetime(),
    max_value=max_timestamp.to_pydatetime(),
    step = timedelta(minutes=1),
    value=(min_timestamp.to_pydatetime(), max_timestamp.to_pydatetime()),
    format='YYYY-MM-DD HH:mm',
)

print(f"S start_timestamp: {start_timestamp}")
print(f"S end_timestamp: {end_timestamp}")

start_timestamp = start_timestamp.timestamp()
end_timestamp = end_timestamp.timestamp()
# Convert timestamp back to datetime for filtering (i.tz_convert('US/Pacific')ncluding microseconds)
# timezone = UTC-8
start_time = pd.to_datetime(start_timestamp, utc=False, unit='s')- timedelta(hours=7)
end_time = pd.to_datetime(end_timestamp, utc=False, unit='s') - timedelta(hours=7)
print(f"start_time: {start_time}")
print(f"end_time: {end_time}")



# Filter the dataframe based on the selected time range
filtered_df = df[(df['timestamp'] >= start_time.to_datetime64()) & (df['timestamp'] <= end_time.to_datetime64())]

# print(filtered_df.head(20))

# Assign a unique color to each MAC address
colors = pd.factorize(filtered_df['mac_address'])[0]
filtered_df['color'] = colors

# Define a PyDeck layer for the GPS data
layer = pdk.Layer(
    "ScatterplotLayer",
    data=filtered_df,
    get_position=["longitude", "latitude"],
    get_color="[color * 30, color * 50, 150]",
    get_radius=1,
)

# Set the view options for the map
view_state = pdk.ViewState(
    latitude=filtered_df['latitude'].mean(),
    longitude=filtered_df['longitude'].mean(),
    zoom=18,
)

# Render the map with the layer
st.pydeck_chart(pdk.Deck(layers=[layer], initial_view_state=view_state))

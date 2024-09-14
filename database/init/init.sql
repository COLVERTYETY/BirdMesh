CREATE TABLE IF NOT EXISTS Measurement (
    measurement_id SERIAL PRIMARY KEY,
    mac_address VARCHAR(4) NOT NULL,
    timestamp TIMESTAMP NOT NULL,
    measurement_type INT NOT NULL,
    measurement_value FLOAT NOT NULL
);

CREATE TABLE IF NOT EXISTS GPS (
    gps_id SERIAL PRIMARY KEY,
    mac_address VARCHAR(4) NOT NULL,
    timestamp TIMESTAMP NOT NULL,
    longitude FLOAT NOT NULL,
    latitude FLOAT NOT NULL,
    n_satellites INT NOT NULL
);

CREATE TABLE IF NOT EXISTS files (
    file_id SERIAL PRIMARY KEY,
    mac_address VARCHAR(4) NOT NULL,
    timestamp TIMESTAMP NOT NULL,
    data BYTEA NOT NULL
);

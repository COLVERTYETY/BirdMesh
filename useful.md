check if database is running: 

```bash
docker exec -it database_postgres_1 psql -U Birds -d BirdMeshDB -c "SELECT version();"
```

list the tables in the database:

```bash
docker exec -it database_postgres_1 psql -U Birds -d BirdMeshDB -c "\dt"
```
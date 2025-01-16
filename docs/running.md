# Running Basestation

> [!NOTE]
> All instructions are intended to be begin in the root directory of the
> `rover-Basestation-Release` repository.

## Backend

To run the backend, first ensure you have the Python dependencies installed:

```bash
# change to the backend directory
cd backend
# install packages
pip install -r requirements.txt
```

Then launch the app:

```bash
# change to the backend directory
cd backend
# run the app
./app.py
```

## Frontend

To build the frontend, run these commands:

```bash
# change to the front
cd frontend
# build the webapp
npm run build
```

## Troubleshooting

If things aren't being served to <http://localhost:80> properly, ensure Caddy is
running:

```bash
sudo caddy start
```

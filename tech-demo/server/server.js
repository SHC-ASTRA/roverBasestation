const express = require("express");
const app = express();

app.get('/api', (req, res) => {
    res.json({"users": ["userOne", "userTwo", "userThree"]})
});

const port = 8080;

app.listen(port, () => {console.log(`Server started on port ${port}`)});
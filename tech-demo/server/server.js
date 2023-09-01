const express = require("express");
const app = express();
const port = 8080;

app.get('/api', (req, res) => {
    // sending data to front end is res.json({data: data});
    res.json({"users": ["userOne", "userTwo", "userThree"]})
});

app.listen(port, () => {console.log(`Server started on port ${port}`)});
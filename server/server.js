const express = require('express');
const app = express();
const port = 8000;

app.get('/api', (req, res) => {
    // communicate through api here
    // res.json({'users': ['userOne', 'userTwo', 'userThree']});
});

app.listen(port, () => {console.log(`Server started on port ${port}`)});
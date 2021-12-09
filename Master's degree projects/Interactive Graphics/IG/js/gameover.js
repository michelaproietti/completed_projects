$(document).ready(function()  {
    console.log(sessionStorage.getItem("passaggioScore"));
    document.getElementById("score").innerHTML = sessionStorage.getItem("passaggioScore");
});
<?php

	$servername = "";
	$username = "";
	$password = "";
	$dbname = "";
	
	$conn = new mysqli($servername, $username, $password, $dbname);
	
	if ($conn->connect_error) {
		die("Connection failed: " . $conn->connect_error);
	} else {
		echo "connection established<br />";
	}

	$orientation = "x: " . $_GET["x"] . " y: " . $_GET["y"] . " z: " . $_GET["z"];
	$temperature = $_GET["t"];
	$rssi = $_GET["rssi"];
	
	// basic checks
	if ($temperature === null) {
		$temperature = 0.0;
	}
	if ($rssi === null) {
		$rssi = 0;
	}
	
	$sql = "INSERT INTO weather (temperature, orientation, rssi) VALUES ('$temperature', '$orientation', '$rssi')";

    if ($conn->query($sql)) {
        echo "Data has been posted to the database\n";
    } else {
        echo "There was an error in posting the data: " . $conn->error;
    }
	
	$conn->close();
	
?>

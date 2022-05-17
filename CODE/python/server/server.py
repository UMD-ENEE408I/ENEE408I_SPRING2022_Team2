

from flask import Flask, jsonify
import json
import base64

app = flask(__name__)
@app.route('/',methods = ["GET"])
def process_image():
       

 if __name__ == "__main__":
    app.debug = True
    app.run(debug = False, host = '0.0.0.0')
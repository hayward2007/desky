import tensorflow as tf
import os

from dotenv import load_dotenv
load_dotenv()


# get env from local.env
port = os.getenv("PORT")
print(f"Port: {port}")
from datasets import load_dataset
from transformers import GPT2TokenizerFast, DataCollatorForLanguageModeling

tokenizer = GPT2TokenizerFast.from_pretrained("./r2d2_tokenizer")

# Cargar dataset
dataset = load_dataset("json", data_files={"train": "r2d2_dataset.json"})

# Tokenizar prompt + completion
def tokenize(example):
    return tokenizer(example["prompt"] + " " + example["completion"], truncation=True, max_length=128)

tokenized_dataset = dataset["train"].map(tokenize, batched=False)

# Guardar dataset tokenizado
tokenized_dataset.save_to_disk("./r2d2_tokenized")
print("Dataset tokenizado listo en ./r2d2_tokenized")


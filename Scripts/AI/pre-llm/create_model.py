from transformers import GPT2Config, GPT2LMHeadModel, GPT2TokenizerFast

# Cargar tokenizer
tokenizer = GPT2TokenizerFast.from_pretrained("./r2d2_tokenizer")

# Configuración ligera (~70MB)
config = GPT2Config(
    vocab_size=len(tokenizer),  # importante: coincide con tokenizer
    n_positions=128,
    n_ctx=128,
    n_embd=256,
    n_layer=4,
    n_head=4
)

model = GPT2LMHeadModel(config)

# Ajustar embeddings si añadiste tokens especiales
model.resize_token_embeddings(len(tokenizer))

# Guardar modelo inicial
model.save_pretrained("./r2d2_model")
tokenizer.save_pretrained("./r2d2_model")
print("Modelo inicial guardado en ./r2d2_model")


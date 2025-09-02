from transformers import GPT2LMHeadModel, GPT2TokenizerFast

tokenizer = GPT2TokenizerFast.from_pretrained("./r2d2_model_trained")
model = GPT2LMHeadModel.from_pretrained("./r2d2_model_trained")

print("¡R2D2 listo para conversar!")

while True:
    user_input = input("Jose: ")

    # Tokenizamos solo la entrada actual
    input_ids = tokenizer(user_input, return_tensors="pt").input_ids

    # Generación independiente, sin historial
    output_ids = model.generate(
        input_ids=input_ids,
        do_sample=True,
        temperature=0.1,
        top_k=50,
        top_p=0.9,
        max_new_tokens=50,
        pad_token_id=tokenizer.eos_token_id,
        no_repeat_ngram_size=3
    )

    # Decodificamos la respuesta y mostramos
    response = tokenizer.decode(output_ids[0][len(input_ids[0]):], skip_special_tokens=True)
    print("R2D2:", response)


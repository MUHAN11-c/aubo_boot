export { messageDefinitionHasDataFields };
function messageDefinitionHasDataFields(fields) {
    return fields.some((field) => field.isConstant !== true);
}
//# sourceMappingURL=messageDefinitionHasDataFields.js.map
// json_mini.hpp — tiny, dependency-free JSON reader for the golden test suite.
//
// Scope: just enough to parse the golden files this repo writes (objects, arrays,
// numbers, strings, booleans, null). NaN is encoded as JSON `null`. This lives under
// tests/ only — the library never depends on it. Not a general-purpose JSON library:
// no unicode escapes beyond the basics, numbers are parsed as double.
#pragma once

#include <cmath>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace jsonmini
{
	struct Value;
	using Array = std::vector<Value>;
	using Member = std::pair<std::string, Value>;
	using Object = std::vector<Member>; // ordered; small, linear lookup is fine

	struct Value
	{
		enum class Type
		{
			Null,
			Bool,
			Number,
			String,
			Array,
			Object
		};

		Type type = Type::Null;
		bool boolean = false;
		double number = 0.0;
		std::string string;
		std::shared_ptr<Array> array;
		std::shared_ptr<Object> object;

		bool isNull() const
		{
			return type == Type::Null;
		}
		float asFloat() const
		{
			return static_cast<float>(number);
		}
		double asDouble() const
		{
			return number;
		}
		int asInt() const
		{
			return static_cast<int>(number);
		}
		bool asBool() const
		{
			return boolean;
		}
		const std::string& asString() const
		{
			return string;
		}
		const Array& asArray() const
		{
			return *array;
		}

		// Object member access by key (throws if absent).
		const Value& operator[](const std::string& key) const
		{
			for (const auto& kv : *object)
				if (kv.first == key)
					return kv.second;
			throw std::runtime_error("json_mini: key not found: " + key);
		}

		// Array element access by index.
		const Value& operator[](std::size_t i) const
		{
			return (*array)[i];
		}

		std::size_t size() const
		{
			return array ? array->size() : 0;
		}
	};

	class Parser
	{
	  public:
		explicit Parser(const std::string& text) : m_text(text)
		{
		}

		Value parse()
		{
			skipWs();
			Value v = parseValue();
			skipWs();
			if (m_pos != m_text.size())
				fail("trailing characters after top-level value");
			return v;
		}

	  private:
		const std::string& m_text;
		std::size_t m_pos = 0;

		[[noreturn]] void fail(const std::string& msg) const
		{
			throw std::runtime_error("json_mini: parse error at " + std::to_string(m_pos) + ": " + msg);
		}

		char peek() const
		{
			return m_pos < m_text.size() ? m_text[m_pos] : '\0';
		}
		char get()
		{
			return m_text[m_pos++];
		}

		void skipWs()
		{
			while (m_pos < m_text.size())
			{
				const char c = m_text[m_pos];
				if (c == ' ' || c == '\t' || c == '\n' || c == '\r')
					++m_pos;
				else
					break;
			}
		}

		void expect(char c)
		{
			if (peek() != c)
				fail(std::string("expected '") + c + "'");
			++m_pos;
		}

		Value parseValue()
		{
			skipWs();
			switch (peek())
			{
			case '{':
				return parseObject();
			case '[':
				return parseArray();
			case '"':
				return parseString();
			case 't':
			case 'f':
				return parseBool();
			case 'n':
				return parseNull();
			default:
				return parseNumber();
			}
		}

		Value parseObject()
		{
			Value v;
			v.type = Value::Type::Object;
			v.object = std::make_shared<Object>();
			expect('{');
			skipWs();
			if (peek() == '}')
			{
				++m_pos;
				return v;
			}
			for (;;)
			{
				skipWs();
				if (peek() != '"')
					fail("expected string key");
				std::string key = parseRawString();
				skipWs();
				expect(':');
				Value val = parseValue();
				v.object->emplace_back(std::move(key), std::move(val));
				skipWs();
				const char c = get();
				if (c == ',')
					continue;
				if (c == '}')
					break;
				fail("expected ',' or '}' in object");
			}
			return v;
		}

		Value parseArray()
		{
			Value v;
			v.type = Value::Type::Array;
			v.array = std::make_shared<Array>();
			expect('[');
			skipWs();
			if (peek() == ']')
			{
				++m_pos;
				return v;
			}
			for (;;)
			{
				v.array->push_back(parseValue());
				skipWs();
				const char c = get();
				if (c == ',')
					continue;
				if (c == ']')
					break;
				fail("expected ',' or ']' in array");
			}
			return v;
		}

		std::string parseRawString()
		{
			expect('"');
			std::string out;
			for (;;)
			{
				if (m_pos >= m_text.size())
					fail("unterminated string");
				char c = get();
				if (c == '"')
					break;
				if (c == '\\')
				{
					char e = get();
					switch (e)
					{
					case '"':
						out.push_back('"');
						break;
					case '\\':
						out.push_back('\\');
						break;
					case '/':
						out.push_back('/');
						break;
					case 'n':
						out.push_back('\n');
						break;
					case 't':
						out.push_back('\t');
						break;
					case 'r':
						out.push_back('\r');
						break;
					case 'b':
						out.push_back('\b');
						break;
					case 'f':
						out.push_back('\f');
						break;
					default:
						fail("unsupported escape");
					}
				}
				else
				{
					out.push_back(c);
				}
			}
			return out;
		}

		Value parseString()
		{
			Value v;
			v.type = Value::Type::String;
			v.string = parseRawString();
			return v;
		}

		Value parseBool()
		{
			Value v;
			v.type = Value::Type::Bool;
			if (m_text.compare(m_pos, 4, "true") == 0)
			{
				v.boolean = true;
				m_pos += 4;
			}
			else if (m_text.compare(m_pos, 5, "false") == 0)
			{
				v.boolean = false;
				m_pos += 5;
			}
			else
				fail("invalid literal");
			return v;
		}

		Value parseNull()
		{
			if (m_text.compare(m_pos, 4, "null") != 0)
				fail("invalid literal");
			m_pos += 4;
			Value v; // Type::Null by default
			return v;
		}

		Value parseNumber()
		{
			const std::size_t start = m_pos;
			if (peek() == '-')
				++m_pos;
			while (std::isdigit(static_cast<unsigned char>(peek())))
				++m_pos;
			if (peek() == '.')
			{
				++m_pos;
				while (std::isdigit(static_cast<unsigned char>(peek())))
					++m_pos;
			}
			if (peek() == 'e' || peek() == 'E')
			{
				++m_pos;
				if (peek() == '+' || peek() == '-')
					++m_pos;
				while (std::isdigit(static_cast<unsigned char>(peek())))
					++m_pos;
			}
			if (m_pos == start)
				fail("invalid number");
			Value v;
			v.type = Value::Type::Number;
			v.number = std::strtod(m_text.c_str() + start, nullptr);
			return v;
		}
	};

	inline Value parse(const std::string& text)
	{
		return Parser(text).parse();
	}
} // namespace jsonmini
